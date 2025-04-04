// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2017-2023 Oracle.  All Rights Reserved.
 * Author: Darrick J. Wong <djwong@kernel.org>
 */
#include "xfs.h"
#include "xfs_fs.h"
#include "xfs_shared.h"
#include "xfs_format.h"
#include "xfs_trans_resv.h"
#include "xfs_mount.h"
#include "xfs_btree.h"
#include "xfs_log_format.h"
#include "xfs_trans.h"
#include "xfs_rtbitmap.h"
#include "xfs_inode.h"
#include "xfs_bmap.h"
#include "xfs_bit.h"
#include "xfs_rtgroup.h"
#include "xfs_sb.h"
#include "xfs_rmap.h"
#include "xfs_rtrmap_btree.h"
#include "xfs_exchmaps.h"
#include "xfs_zone_alloc.h"
#include "scrub/scrub.h"
#include "scrub/common.h"
#include "scrub/repair.h"
#include "scrub/tempexch.h"
#include "scrub/rtbitmap.h"
#include "scrub/btree.h"

/* Set us up with the realtime metadata locked. */
int
xchk_setup_rtbitmap(
	struct xfs_scrub	*sc)
{
	struct xfs_mount	*mp = sc->mp;
	struct xchk_rtbitmap	*rtb;
	int			error;

	if (xchk_need_intent_drain(sc))
		xchk_fsgates_enable(sc, XCHK_FSGATES_DRAIN);

	rtb = kzalloc(struct_size(rtb, words, xchk_rtbitmap_wordcnt(sc)),
			XCHK_GFP_FLAGS);
	if (!rtb)
		return -ENOMEM;
	sc->buf = rtb;
	rtb->sc = sc;

	error = xchk_rtgroup_init(sc, sc->sm->sm_agno, &sc->sr);
	if (error)
		return error;

	if (xchk_could_repair(sc)) {
		error = xrep_setup_rtbitmap(sc, rtb);
		if (error)
			return error;
	}

	error = xchk_trans_alloc(sc, rtb->resblks);
	if (error)
		return error;

	error = xchk_install_live_inode(sc, rtg_bitmap(sc->sr.rtg));
	if (error)
		return error;

	error = xchk_ino_dqattach(sc);
	if (error)
		return error;

	error = xchk_rtgroup_lock(sc, &sc->sr, XCHK_RTGLOCK_ALL);
	if (error)
		return error;

	/*
	 * Now that we've locked the rtbitmap, we can't race with growfsrt
	 * trying to expand the bitmap or change the size of the rt volume.
	 * Hence it is safe to compute and check the geometry values.
	 */
	if (mp->m_sb.sb_rblocks) {
		rtb->rextents = xfs_blen_to_rtbxlen(mp, mp->m_sb.sb_rblocks);
		rtb->rextslog = xfs_compute_rextslog(rtb->rextents);
		rtb->rbmblocks = xfs_rtbitmap_blockcount(mp);
	}

	return 0;
}

/* Per-rtgroup bitmap contents. */

/* Cross-reference rtbitmap entries with other metadata. */
STATIC void
xchk_rtbitmap_xref(
	struct xchk_rtbitmap	*rtb,
	xfs_rtblock_t		startblock,
	xfs_rtblock_t		blockcount)
{
	struct xfs_scrub	*sc = rtb->sc;
	xfs_rgblock_t		rgbno = xfs_rtb_to_rgbno(sc->mp, startblock);

	if (sc->sm->sm_flags & XFS_SCRUB_OFLAG_CORRUPT)
		return;
	if (!sc->sr.rmap_cur)
		return;

	xchk_xref_has_no_rt_owner(sc, rgbno, blockcount);
	xchk_xref_is_not_rt_shared(sc, rgbno, blockcount);
	xchk_xref_is_not_rt_cow_staging(sc, rgbno, blockcount);

	if (rtb->next_free_rgbno < rgbno)
		xchk_xref_has_rt_owner(sc, rtb->next_free_rgbno,
				rgbno - rtb->next_free_rgbno);
	rtb->next_free_rgbno = rgbno + blockcount;
}

/* Scrub a free extent record from the realtime bitmap. */
STATIC int
xchk_rtbitmap_rec(
	struct xfs_rtgroup	*rtg,
	struct xfs_trans	*tp,
	const struct xfs_rtalloc_rec *rec,
	void			*priv)
{
	struct xchk_rtbitmap	*rtb = priv;
	struct xfs_scrub	*sc = rtb->sc;
	xfs_rtblock_t		startblock;
	xfs_filblks_t		blockcount;

	startblock = xfs_rtx_to_rtb(rtg, rec->ar_startext);
	blockcount = xfs_rtxlen_to_extlen(rtg_mount(rtg), rec->ar_extcount);

	if (!xfs_verify_rtbext(rtg_mount(rtg), startblock, blockcount))
		xchk_fblock_set_corrupt(sc, XFS_DATA_FORK, 0);

	xchk_rtbitmap_xref(rtb, startblock, blockcount);

	if (sc->sm->sm_flags & XFS_SCRUB_OFLAG_CORRUPT)
		return -ECANCELED;

	return 0;
}

/* Make sure the entire rtbitmap file is mapped with written extents. */
STATIC int
xchk_rtbitmap_check_extents(
	struct xfs_scrub	*sc)
{
	struct xfs_bmbt_irec	map;
	struct xfs_iext_cursor	icur;
	struct xfs_mount	*mp = sc->mp;
	struct xfs_inode	*ip = sc->ip;
	xfs_fileoff_t		off = 0;
	xfs_fileoff_t		endoff;
	int			error = 0;

	/* Mappings may not cross or lie beyond EOF. */
	endoff = XFS_B_TO_FSB(mp, ip->i_disk_size);
	if (xfs_iext_lookup_extent(ip, &ip->i_df, endoff, &icur, &map)) {
		xchk_fblock_set_corrupt(sc, XFS_DATA_FORK, endoff);
		return 0;
	}

	while (off < endoff) {
		int		nmap = 1;

		if (xchk_should_terminate(sc, &error) ||
		    (sc->sm->sm_flags & XFS_SCRUB_OFLAG_CORRUPT))
			break;

		/* Make sure we have a written extent. */
		error = xfs_bmapi_read(ip, off, endoff - off, &map, &nmap,
				XFS_DATA_FORK);
		if (!xchk_fblock_process_error(sc, XFS_DATA_FORK, off, &error))
			break;

		if (nmap != 1 || !xfs_bmap_is_written_extent(&map)) {
			xchk_fblock_set_corrupt(sc, XFS_DATA_FORK, off);
			break;
		}

		off += map.br_blockcount;
	}

	return error;
}

/* Scrub this group's realtime bitmap. */
int
xchk_rtbitmap(
	struct xfs_scrub	*sc)
{
	struct xfs_mount	*mp = sc->mp;
	struct xfs_rtgroup	*rtg = sc->sr.rtg;
	struct xfs_inode	*rbmip = rtg_bitmap(rtg);
	struct xchk_rtbitmap	*rtb = sc->buf;
	xfs_rgblock_t		last_rgbno;
	int			error;

	/* Is sb_rextents correct? */
	if (mp->m_sb.sb_rextents != rtb->rextents) {
		xchk_ino_set_corrupt(sc, rbmip->i_ino);
		return 0;
	}

	/* Is sb_rextslog correct? */
	if (mp->m_sb.sb_rextslog != rtb->rextslog) {
		xchk_ino_set_corrupt(sc, rbmip->i_ino);
		return 0;
	}

	/*
	 * Is sb_rbmblocks large enough to handle the current rt volume?  In no
	 * case can we exceed 4bn bitmap blocks since the super field is a u32.
	 */
	if (rtb->rbmblocks > U32_MAX) {
		xchk_ino_set_corrupt(sc, rbmip->i_ino);
		return 0;
	}
	if (mp->m_sb.sb_rbmblocks != rtb->rbmblocks) {
		xchk_ino_set_corrupt(sc, rbmip->i_ino);
		return 0;
	}

	/* The bitmap file length must be aligned to an fsblock. */
	if (rbmip->i_disk_size & mp->m_blockmask) {
		xchk_ino_set_corrupt(sc, rbmip->i_ino);
		return 0;
	}

	/*
	 * Is the bitmap file itself large enough to handle the rt volume?
	 * growfsrt expands the bitmap file before updating sb_rextents, so the
	 * file can be larger than sb_rbmblocks.
	 */
	if (rbmip->i_disk_size < XFS_FSB_TO_B(mp, rtb->rbmblocks)) {
		xchk_ino_set_corrupt(sc, rbmip->i_ino);
		return 0;
	}

	/* Invoke the fork scrubber. */
	error = xchk_metadata_inode_forks(sc);
	if (error || (sc->sm->sm_flags & XFS_SCRUB_OFLAG_CORRUPT))
		return error;

	error = xchk_rtbitmap_check_extents(sc);
	if (error || (sc->sm->sm_flags & XFS_SCRUB_OFLAG_CORRUPT))
		return error;

	rtb->next_free_rgbno = 0;
	error = xfs_rtalloc_query_all(rtg, sc->tp, xchk_rtbitmap_rec, rtb);
	if (!xchk_fblock_process_error(sc, XFS_DATA_FORK, 0, &error))
		return error;

	/*
	 * Check that the are rmappings for all rt extents between the end of
	 * the last free extent we saw and the last possible extent in the rt
	 * group.
	 */
	last_rgbno = rtg->rtg_extents * mp->m_sb.sb_rextsize - 1;
	if (rtb->next_free_rgbno < last_rgbno)
		xchk_xref_has_rt_owner(sc, rtb->next_free_rgbno,
				last_rgbno - rtb->next_free_rgbno);
	return 0;
}

/* xref check that the extent is not free in the rtbitmap */
void
xchk_xref_is_used_rt_space(
	struct xfs_scrub	*sc,
	xfs_rtblock_t		rtbno,
	xfs_extlen_t		len)
{
	struct xfs_rtgroup	*rtg = sc->sr.rtg;
	xfs_rtxnum_t		startext;
	xfs_rtxnum_t		endext;
	bool			is_free;
	int			error;

	if (xchk_skip_xref(sc->sm))
		return;

	if (xfs_has_zoned(sc->mp)) {
		if (!xfs_zone_rgbno_is_valid(rtg,
				xfs_rtb_to_rgbno(sc->mp, rtbno) + len - 1))
			xchk_ino_xref_set_corrupt(sc, rtg_rmap(rtg)->i_ino);
		return;
	}

	startext = xfs_rtb_to_rtx(sc->mp, rtbno);
	endext = xfs_rtb_to_rtx(sc->mp, rtbno + len - 1);
	error = xfs_rtalloc_extent_is_free(rtg, sc->tp, startext,
			endext - startext + 1, &is_free);
	if (!xchk_should_check_xref(sc, &error, NULL))
		return;
	if (is_free)
		xchk_ino_xref_set_corrupt(sc, rtg_bitmap(rtg)->i_ino);
}
