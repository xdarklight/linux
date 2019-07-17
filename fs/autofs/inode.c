// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright 1997-1998 Transmeta Corporation -- All Rights Reserved
 * Copyright 2005-2006 Ian Kent <raven@themaw.net>
 */

#include <linux/seq_file.h>
#include <linux/pagemap.h>
#include <linux/fs_context.h>
#include <linux/fs_parser.h>

#include "autofs_i.h"

struct autofs_info *autofs_new_ino(struct autofs_sb_info *sbi)
{
	struct autofs_info *ino;

	ino = kzalloc(sizeof(*ino), GFP_KERNEL);
	if (ino) {
		INIT_LIST_HEAD(&ino->active);
		INIT_LIST_HEAD(&ino->expiring);
		ino->last_used = jiffies;
		ino->sbi = sbi;
	}
	return ino;
}

void autofs_clean_ino(struct autofs_info *ino)
{
	ino->uid = GLOBAL_ROOT_UID;
	ino->gid = GLOBAL_ROOT_GID;
	ino->last_used = jiffies;
}

void autofs_free_ino(struct autofs_info *ino)
{
	kfree_rcu(ino, rcu);
}

void autofs_kill_sb(struct super_block *sb)
{
	struct autofs_sb_info *sbi = autofs_sbi(sb);

	/*
	 * In the event of a failure in get_sb_nodev the superblock
	 * info is not present so nothing else has been setup, so
	 * just call kill_anon_super when we are called from
	 * deactivate_super.
	 */
	if (sbi) {
		/* Free wait queues, close pipe */
		autofs_catatonic_mode(sbi);
		put_pid(sbi->oz_pgrp);
	}

	pr_debug("shutting down\n");
	kill_litter_super(sb);
	if (sbi)
		kfree_rcu(sbi, rcu);
}

static int autofs_show_options(struct seq_file *m, struct dentry *root)
{
	struct autofs_sb_info *sbi = autofs_sbi(root->d_sb);
	struct inode *root_inode = d_inode(root->d_sb->s_root);

	if (!sbi)
		return 0;

	seq_printf(m, ",fd=%d", sbi->pipefd);
	if (!uid_eq(root_inode->i_uid, GLOBAL_ROOT_UID))
		seq_printf(m, ",uid=%u",
			from_kuid_munged(&init_user_ns, root_inode->i_uid));
	if (!gid_eq(root_inode->i_gid, GLOBAL_ROOT_GID))
		seq_printf(m, ",gid=%u",
			from_kgid_munged(&init_user_ns, root_inode->i_gid));
	seq_printf(m, ",pgrp=%d", pid_vnr(sbi->oz_pgrp));
	seq_printf(m, ",timeout=%lu", sbi->exp_timeout/HZ);
	seq_printf(m, ",minproto=%d", sbi->min_proto);
	seq_printf(m, ",maxproto=%d", sbi->max_proto);

	if (autofs_type_offset(sbi->type))
		seq_puts(m, ",offset");
	else if (autofs_type_direct(sbi->type))
		seq_puts(m, ",direct");
	else
		seq_puts(m, ",indirect");
	if (sbi->flags & AUTOFS_SBI_STRICTEXPIRE)
		seq_puts(m, ",strictexpire");
	if (sbi->flags & AUTOFS_SBI_IGNORE)
		seq_puts(m, ",ignore");
#ifdef CONFIG_CHECKPOINT_RESTORE
	if (sbi->pipe)
		seq_printf(m, ",pipe_ino=%ld", file_inode(sbi->pipe)->i_ino);
	else
		seq_puts(m, ",pipe_ino=-1");
#endif
	return 0;
}

static void autofs_evict_inode(struct inode *inode)
{
	clear_inode(inode);
	kfree(inode->i_private);
}

static const struct super_operations autofs_sops = {
	.statfs		= simple_statfs,
	.show_options	= autofs_show_options,
	.evict_inode	= autofs_evict_inode,
};

struct autofs_fs_context {
	kuid_t	uid;
	kgid_t	gid;
	};

enum {
	Opt_direct,
	Opt_fd,
	Opt_gid,
	Opt_ignore,
	Opt_indirect,
	Opt_maxproto,
	Opt_minproto,
	Opt_offset,
	Opt_pgrp,
	Opt_strictexpire,
	Opt_uid,
};

static const struct fs_parameter_spec autofs_param_specs[] = {
	fsparam_flag	("direct",			Opt_direct),
	fsparam_fd	("fd",				Opt_fd),
	fsparam_u32	("gid",				Opt_gid),
	fsparam_flag	("ignore",			Opt_ignore),
	fsparam_flag	("indirect",			Opt_indirect),
	fsparam_u32	("maxproto",			Opt_maxproto),
	fsparam_u32	("minproto",			Opt_minproto),
	fsparam_flag	("offset",			Opt_offset),
	fsparam_u32	("pgrp",			Opt_pgrp),
	fsparam_flag	("strictexpire",		Opt_strictexpire),
	fsparam_u32	("uid",				Opt_uid),
	{}
};

const struct fs_parameter_description autofs_fs_parameters = {
	.name		= "autofs",
	.specs		= autofs_param_specs,
};

/*
 * Open the fd.  We do it here rather than in get_tree so that it's done in the
 * context of the system call that passed the data and not the one that
 * triggered the superblock creation, lest the fd gets reassigned.
 */
static int autofs_parse_fd(struct fs_context *fc, int pipefd)
{
	struct autofs_sb_info *sbi = fc->s_fs_info;
	struct file *pipe;
	int ret;

	pipe = fget(pipefd);
	if (!pipe) {
		errorf(fc, "Pipe file descriptor not open");
		return -EBADF;
	}

	ret = autofs_check_pipe(pipe);
	if (ret < 0) {
		fput(pipe);
		return invalf(fc, "Invalid/unusable pipe");
	}

	sbi->pipefd = pipefd;
	sbi->pipe = pipe;

	return 0;
}

static int autofs_parse_param(struct fs_context *fc, struct fs_parameter *param)
{
	struct autofs_fs_context *ctx = fc->fs_private;
	struct autofs_sb_info *sbi = fc->s_fs_info;
	struct fs_parse_result result;
	struct pid *pgrp;
	kuid_t uid;
	kgid_t gid;
	int opt;

	opt = fs_parse(fc, &autofs_fs_parameters, param, &result);
	if (opt < 0)
		return opt;

	switch (opt) {
	case Opt_fd:
		return autofs_parse_fd(fc, result.int_32);
	case Opt_uid:
		uid = make_kuid(current_user_ns(), result.uint_32);
		if (!uid_valid(uid))
			return 1;
		ctx->uid = uid;
		break;
	case Opt_gid:
		gid = make_kgid(current_user_ns(), result.uint_32);
		if (!gid_valid(gid))
			return 1;
		ctx->gid = gid;
		break;
	case Opt_pgrp:
		pgrp = find_get_pid(result.uint_32);
		if (!pgrp)
			return invalf(fc, "Could not find process group %u",
				      result.uint_32);
		put_pid(sbi->oz_pgrp);
		sbi->oz_pgrp = pgrp;
		break;
	case Opt_minproto:
		sbi->min_proto = result.uint_32;
		break;
	case Opt_maxproto:
		sbi->max_proto = result.uint_32;
		break;
	case Opt_indirect:
		set_autofs_type_indirect(&sbi->type);
		break;
	case Opt_direct:
		set_autofs_type_direct(&sbi->type);
		break;
	case Opt_offset:
		set_autofs_type_offset(&sbi->type);
		break;
	case Opt_strictexpire:
		sbi->flags |= AUTOFS_SBI_STRICTEXPIRE;
		break;
	case Opt_ignore:
		sbi->flags |= AUTOFS_SBI_IGNORE;
		break;
	}

	return 0;
}

static int autofs_fill_super(struct super_block *s, struct fs_context *fc)
{
	struct autofs_fs_context *ctx = fc->fs_private;
	struct autofs_sb_info *sbi = s->s_fs_info;
	struct autofs_info *ino;
	struct inode *root_inode;
	struct dentry *root;

	pr_debug("starting up, sbi = %p\n", sbi);

	sbi->sb = s;
	s->s_blocksize = 1024;
	s->s_blocksize_bits = 10;
	s->s_magic = AUTOFS_SUPER_MAGIC;
	s->s_op = &autofs_sops;
	s->s_d_op = &autofs_dentry_operations;
	s->s_time_gran = 1;

	/*
	 * Get the root inode and dentry.
	 */
	ino = autofs_new_ino(sbi);
	if (!ino)
		goto nomem;

	root_inode = autofs_get_inode(s, S_IFDIR | 0755);
	root_inode->i_uid = ctx->uid;
	root_inode->i_gid = ctx->gid;
	root_inode->i_fop = &autofs_root_operations;
	root_inode->i_op = &autofs_dir_inode_operations;

	root = d_make_root(root_inode);
	if (!root)
		goto nomem_ino;

	root->d_fsdata = ino;

	if (autofs_type_trigger(sbi->type))
		__managed_dentry_set_managed(root);

	pr_debug("pipe fd = %d, pgrp = %u\n",
		 sbi->pipefd, pid_nr(sbi->oz_pgrp));

	autofs_prepare_pipe(sbi->pipe);

	sbi->flags &= ~AUTOFS_SBI_CATATONIC;

	/*
	 * Success! Install the root dentry now to indicate completion.
	 */
	s->s_root = root;
	return 0;

	/*
	 * Failure ... clean up.
	 */
nomem_ino:
	autofs_free_ino(ino);
nomem:
	return -ENOMEM;
}

/*
 * Validate the parameters and then request a superblock.
 */
static int autofs_get_tree(struct fs_context *fc)
{
	struct autofs_sb_info *sbi = fc->s_fs_info;

	/* Test versions first */
	if (sbi->max_proto < AUTOFS_MIN_PROTO_VERSION ||
	    sbi->min_proto > AUTOFS_MAX_PROTO_VERSION)
		return invalf(fc, "kernel does not match daemon version "
			      "daemon (%d, %d) kernel (%d, %d)\n",
			      sbi->min_proto, sbi->max_proto,
			      AUTOFS_MIN_PROTO_VERSION, AUTOFS_MAX_PROTO_VERSION);

	/* Establish highest kernel protocol version */
	if (sbi->max_proto > AUTOFS_MAX_PROTO_VERSION)
		sbi->version = AUTOFS_MAX_PROTO_VERSION;
	else
		sbi->version = sbi->max_proto;
	sbi->sub_version = AUTOFS_PROTO_SUBVERSION;

	if (!sbi->pipe)
		return invalf(fc, "No control pipe specified");

	return get_tree_nodev(fc, autofs_fill_super);
}

static void autofs_free_fc(struct fs_context *fc)
{
	struct autofs_fs_context *ctx = fc->fs_private;
	struct autofs_sb_info *sbi = fc->s_fs_info;

	if (sbi) {
		if (sbi->pipe)
			fput(sbi->pipe);
		put_pid(sbi->oz_pgrp);
		kfree(sbi);
	}
	kfree(ctx);
}

static const struct fs_context_operations autofs_context_ops = {
	.free		= autofs_free_fc,
	.parse_param	= autofs_parse_param,
	.get_tree	= autofs_get_tree,
};

/*
 * Set up the filesystem mount context.
 */
int autofs_init_fs_context(struct fs_context *fc)
{
	struct autofs_fs_context *ctx;
	struct autofs_sb_info *sbi;

	ctx = kzalloc(sizeof(struct autofs_fs_context), GFP_KERNEL);
	if (!ctx)
		goto nomem;

	ctx->uid = current_uid();
	ctx->gid = current_gid();

	sbi = kzalloc(sizeof(struct autofs_sb_info), GFP_KERNEL);
	if (!sbi)
		goto nomem_ctx;

	sbi->magic = AUTOFS_SBI_MAGIC;
	sbi->flags = AUTOFS_SBI_CATATONIC;
	sbi->min_proto = AUTOFS_MIN_PROTO_VERSION;
	sbi->max_proto = AUTOFS_MAX_PROTO_VERSION;
	sbi->pipefd = -1;
	sbi->oz_pgrp = get_task_pid(current, PIDTYPE_PGID);

	set_autofs_type_indirect(&sbi->type);
	mutex_init(&sbi->wq_mutex);
	mutex_init(&sbi->pipe_mutex);
	spin_lock_init(&sbi->fs_lock);
	spin_lock_init(&sbi->lookup_lock);
	INIT_LIST_HEAD(&sbi->active_list);
	INIT_LIST_HEAD(&sbi->expiring_list);

	fc->fs_private = ctx;
	fc->s_fs_info = sbi;
	fc->ops = &autofs_context_ops;
	return 0;

nomem_ctx:
	kfree(ctx);
nomem:
	return -ENOMEM;
}

struct inode *autofs_get_inode(struct super_block *sb, umode_t mode)
{
	struct inode *inode = new_inode(sb);

	if (inode == NULL)
		return NULL;

	inode->i_mode = mode;
	if (sb->s_root) {
		inode->i_uid = d_inode(sb->s_root)->i_uid;
		inode->i_gid = d_inode(sb->s_root)->i_gid;
	}
	inode->i_atime = inode->i_mtime = inode->i_ctime = current_time(inode);
	inode->i_ino = get_next_ino();

	if (S_ISDIR(mode)) {
		set_nlink(inode, 2);
		inode->i_op = &autofs_dir_inode_operations;
		inode->i_fop = &autofs_dir_operations;
	} else if (S_ISLNK(mode)) {
		inode->i_op = &autofs_symlink_inode_operations;
	} else
		WARN_ON(1);

	return inode;
}
