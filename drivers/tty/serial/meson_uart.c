// SPDX-License-Identifier: GPL-2.0
/*
 *  Based on meson_uart.c, by AMLOGIC, INC.
 *
 * Copyright (C) 2014 Carlo Caione <carlo@caione.org>
 */

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/console.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/serial.h>
#include <linux/serial_core.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>

/* Register offsets */
#define AML_UART_WFIFO			0x00
#define AML_UART_RFIFO			0x04
#define AML_UART_CONTROL		0x08
#define AML_UART_STATUS			0x0c
#define AML_UART_MISC			0x10
#define AML_UART_REG5			0x14

/* AML_UART_CONTROL bits */
#define AML_UART_TX_EN			BIT(12)
#define AML_UART_RX_EN			BIT(13)
#define AML_UART_TWO_WIRE_EN		BIT(15)
#define AML_UART_STOP_BIT_LEN_MASK	(0x03 << 16)
#define AML_UART_STOP_BIT_1SB		(0x00 << 16)
#define AML_UART_STOP_BIT_2SB		(0x01 << 16)
#define AML_UART_PARITY_TYPE		BIT(18)
#define AML_UART_PARITY_EN		BIT(19)
#define AML_UART_TX_RST			BIT(22)
#define AML_UART_RX_RST			BIT(23)
#define AML_UART_CLEAR_ERR		BIT(24)
#define AML_UART_RX_INT_EN		BIT(27)
#define AML_UART_TX_INT_EN		BIT(28)
#define AML_UART_DATA_LEN_MASK		(0x03 << 20)
#define AML_UART_DATA_LEN_8BIT		(0x00 << 20)
#define AML_UART_DATA_LEN_7BIT		(0x01 << 20)
#define AML_UART_DATA_LEN_6BIT		(0x02 << 20)
#define AML_UART_DATA_LEN_5BIT		(0x03 << 20)

/* AML_UART_STATUS bits */
#define AML_UART_PARITY_ERR		BIT(16)
#define AML_UART_FRAME_ERR		BIT(17)
#define AML_UART_TX_FIFO_WERR		BIT(18)
#define AML_UART_RX_EMPTY		BIT(20)
#define AML_UART_TX_FULL		BIT(21)
#define AML_UART_TX_EMPTY		BIT(22)
#define AML_UART_XMIT_BUSY		BIT(25)
#define AML_UART_ERR			(AML_UART_PARITY_ERR | \
					 AML_UART_FRAME_ERR  | \
					 AML_UART_TX_FIFO_WERR)

/* AML_UART_MISC bits */
#define AML_UART_XMIT_IRQ(c)		(((c) & 0xff) << 8)
#define AML_UART_RECV_IRQ(c)		((c) & 0xff)

/* AML_UART_REG5 bits */
#define AML_UART_BAUD_USE		BIT(23)

#define AML_UART_PORT_NUM		12
#define AML_UART_PORT_OFFSET		6
#define AML_UART_DEV_NAME		"ttyAML"

#define AML_UART_POLL_USEC		5
#define AML_UART_TIMEOUT_USEC		10000

struct meson_uart_data {
	struct uart_port	port;
	struct clk		*pclk;
	struct clk		*baud_clk;
	struct clk_divider	baud_div;
	struct clk_mux		use_xtal_mux;
	struct clk_mux		xtal_clk_sel_mux;
	struct clk_mux		xtal2_clk_sel_mux;
	struct clk_fixed_factor	xtal_div2;
	struct clk_fixed_factor	xtal_div3;
	struct clk_fixed_factor	clk81_div4;
	bool			xtal_input_only;
	bool			has_xtal_clk_sel;
};

static struct uart_driver meson_uart_driver;

static struct uart_port *meson_ports[AML_UART_PORT_NUM];

static void meson_uart_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
}

static unsigned int meson_uart_get_mctrl(struct uart_port *port)
{
	return TIOCM_CTS;
}

static unsigned int meson_uart_tx_empty(struct uart_port *port)
{
	u32 val;

	val = readl(port->membase + AML_UART_STATUS);
	val &= (AML_UART_TX_EMPTY | AML_UART_XMIT_BUSY);
	return (val == AML_UART_TX_EMPTY) ? TIOCSER_TEMT : 0;
}

static void meson_uart_stop_tx(struct uart_port *port)
{
	u32 val;

	val = readl(port->membase + AML_UART_CONTROL);
	val &= ~AML_UART_TX_INT_EN;
	writel(val, port->membase + AML_UART_CONTROL);
}

static void meson_uart_stop_rx(struct uart_port *port)
{
	u32 val;

	val = readl(port->membase + AML_UART_CONTROL);
	val &= ~AML_UART_RX_EN;
	writel(val, port->membase + AML_UART_CONTROL);
}

static void meson_uart_shutdown(struct uart_port *port)
{
	unsigned long flags;
	u32 val;

	free_irq(port->irq, port);

	spin_lock_irqsave(&port->lock, flags);

	val = readl(port->membase + AML_UART_CONTROL);
	val &= ~AML_UART_RX_EN;
	val &= ~(AML_UART_RX_INT_EN | AML_UART_TX_INT_EN);
	writel(val, port->membase + AML_UART_CONTROL);

	spin_unlock_irqrestore(&port->lock, flags);
}

static void meson_uart_start_tx(struct uart_port *port)
{
	struct circ_buf *xmit = &port->state->xmit;
	unsigned int ch;
	u32 val;

	if (uart_tx_stopped(port)) {
		meson_uart_stop_tx(port);
		return;
	}

	while (!(readl(port->membase + AML_UART_STATUS) & AML_UART_TX_FULL)) {
		if (port->x_char) {
			writel(port->x_char, port->membase + AML_UART_WFIFO);
			port->icount.tx++;
			port->x_char = 0;
			continue;
		}

		if (uart_circ_empty(xmit))
			break;

		ch = xmit->buf[xmit->tail];
		writel(ch, port->membase + AML_UART_WFIFO);
		xmit->tail = (xmit->tail+1) & (SERIAL_XMIT_SIZE - 1);
		port->icount.tx++;
	}

	if (!uart_circ_empty(xmit)) {
		val = readl(port->membase + AML_UART_CONTROL);
		val |= AML_UART_TX_INT_EN;
		writel(val, port->membase + AML_UART_CONTROL);
	}

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(port);
}

static void meson_receive_chars(struct uart_port *port)
{
	struct tty_port *tport = &port->state->port;
	char flag;
	u32 ostatus, status, ch, mode;

	do {
		flag = TTY_NORMAL;
		port->icount.rx++;
		ostatus = status = readl(port->membase + AML_UART_STATUS);

		if (status & AML_UART_ERR) {
			if (status & AML_UART_TX_FIFO_WERR)
				port->icount.overrun++;
			else if (status & AML_UART_FRAME_ERR)
				port->icount.frame++;
			else if (status & AML_UART_PARITY_ERR)
				port->icount.frame++;

			mode = readl(port->membase + AML_UART_CONTROL);
			mode |= AML_UART_CLEAR_ERR;
			writel(mode, port->membase + AML_UART_CONTROL);

			/* It doesn't clear to 0 automatically */
			mode &= ~AML_UART_CLEAR_ERR;
			writel(mode, port->membase + AML_UART_CONTROL);

			status &= port->read_status_mask;
			if (status & AML_UART_FRAME_ERR)
				flag = TTY_FRAME;
			else if (status & AML_UART_PARITY_ERR)
				flag = TTY_PARITY;
		}

		ch = readl(port->membase + AML_UART_RFIFO);
		ch &= 0xff;

		if ((ostatus & AML_UART_FRAME_ERR) && (ch == 0)) {
			port->icount.brk++;
			flag = TTY_BREAK;
			if (uart_handle_break(port))
				continue;
		}

		if (uart_handle_sysrq_char(port, ch))
			continue;

		if ((status & port->ignore_status_mask) == 0)
			tty_insert_flip_char(tport, ch, flag);

		if (status & AML_UART_TX_FIFO_WERR)
			tty_insert_flip_char(tport, 0, TTY_OVERRUN);

	} while (!(readl(port->membase + AML_UART_STATUS) & AML_UART_RX_EMPTY));

	tty_flip_buffer_push(tport);
}

static irqreturn_t meson_uart_interrupt(int irq, void *dev_id)
{
	struct uart_port *port = (struct uart_port *)dev_id;

	spin_lock(&port->lock);

	if (!(readl(port->membase + AML_UART_STATUS) & AML_UART_RX_EMPTY))
		meson_receive_chars(port);

	if (!(readl(port->membase + AML_UART_STATUS) & AML_UART_TX_FULL)) {
		if (readl(port->membase + AML_UART_CONTROL) & AML_UART_TX_INT_EN)
			meson_uart_start_tx(port);
	}

	spin_unlock(&port->lock);

	return IRQ_HANDLED;
}

static const char *meson_uart_type(struct uart_port *port)
{
	return (port->type == PORT_MESON) ? "meson_uart" : NULL;
}

static void meson_uart_reset(struct uart_port *port)
{
	u32 val;

	val = readl(port->membase + AML_UART_CONTROL);
	val |= (AML_UART_RX_RST | AML_UART_TX_RST | AML_UART_CLEAR_ERR);
	writel(val, port->membase + AML_UART_CONTROL);

	val &= ~(AML_UART_RX_RST | AML_UART_TX_RST | AML_UART_CLEAR_ERR);
	writel(val, port->membase + AML_UART_CONTROL);
}

static int meson_uart_startup(struct uart_port *port)
{
	u32 val;
	int ret;

	meson_uart_reset(port);

	val = readl(port->membase + AML_UART_CONTROL);
	val |= (AML_UART_RX_EN | AML_UART_TX_EN);
	writel(val, port->membase + AML_UART_CONTROL);

	val |= (AML_UART_RX_INT_EN | AML_UART_TX_INT_EN);
	writel(val, port->membase + AML_UART_CONTROL);

	val = (AML_UART_RECV_IRQ(1) | AML_UART_XMIT_IRQ(port->fifosize / 2));
	writel(val, port->membase + AML_UART_MISC);

	ret = request_irq(port->irq, meson_uart_interrupt, 0,
			  port->name, port);

	return ret;
}

static void meson_uart_change_speed(struct uart_port *port, unsigned long baud)
{
	struct meson_uart_data *private_data = port->private_data;
	u32 val;

	while (!meson_uart_tx_empty(port))
		cpu_relax();

	val = readl(port->membase + AML_UART_REG5);
	val |= AML_UART_BAUD_USE;
	writel(val, port->membase + AML_UART_REG5);

	clk_set_rate(private_data->baud_clk, baud);
}

static void meson_uart_set_termios(struct uart_port *port,
				   struct ktermios *termios,
				   struct ktermios *old)
{
	unsigned int cflags, iflags, baud;
	unsigned long flags;
	u32 val;

	spin_lock_irqsave(&port->lock, flags);

	cflags = termios->c_cflag;
	iflags = termios->c_iflag;

	val = readl(port->membase + AML_UART_CONTROL);

	val &= ~AML_UART_DATA_LEN_MASK;
	switch (cflags & CSIZE) {
	case CS8:
		val |= AML_UART_DATA_LEN_8BIT;
		break;
	case CS7:
		val |= AML_UART_DATA_LEN_7BIT;
		break;
	case CS6:
		val |= AML_UART_DATA_LEN_6BIT;
		break;
	case CS5:
		val |= AML_UART_DATA_LEN_5BIT;
		break;
	}

	if (cflags & PARENB)
		val |= AML_UART_PARITY_EN;
	else
		val &= ~AML_UART_PARITY_EN;

	if (cflags & PARODD)
		val |= AML_UART_PARITY_TYPE;
	else
		val &= ~AML_UART_PARITY_TYPE;

	val &= ~AML_UART_STOP_BIT_LEN_MASK;
	if (cflags & CSTOPB)
		val |= AML_UART_STOP_BIT_2SB;
	else
		val |= AML_UART_STOP_BIT_1SB;

	if (cflags & CRTSCTS)
		val &= ~AML_UART_TWO_WIRE_EN;
	else
		val |= AML_UART_TWO_WIRE_EN;

	writel(val, port->membase + AML_UART_CONTROL);

	baud = uart_get_baud_rate(port, termios, old, 50, 4000000);
	meson_uart_change_speed(port, baud);

	port->read_status_mask = AML_UART_TX_FIFO_WERR;
	if (iflags & INPCK)
		port->read_status_mask |= AML_UART_PARITY_ERR |
					  AML_UART_FRAME_ERR;

	port->ignore_status_mask = 0;
	if (iflags & IGNPAR)
		port->ignore_status_mask |= AML_UART_PARITY_ERR |
					    AML_UART_FRAME_ERR;

	uart_update_timeout(port, termios->c_cflag, baud);
	spin_unlock_irqrestore(&port->lock, flags);
}

static int meson_uart_verify_port(struct uart_port *port,
				  struct serial_struct *ser)
{
	int ret = 0;

	if (port->type != PORT_MESON)
		ret = -EINVAL;
	if (port->irq != ser->irq)
		ret = -EINVAL;
	if (ser->baud_base < 9600)
		ret = -EINVAL;
	return ret;
}

static void meson_uart_release_port(struct uart_port *port)
{
	struct meson_uart_data *private_data = port->private_data;

	clk_disable_unprepare(private_data->baud_clk);
	clk_disable_unprepare(private_data->pclk);

	// TODO: devm_clk_hw_put(port->dev, private_data->baud_clk);

	devm_clk_hw_unregister(port->dev, &private_data->baud_div.hw);
	devm_clk_hw_unregister(port->dev, &private_data->use_xtal_mux.hw);

	if (private_data->has_xtal_clk_sel) {
		devm_clk_hw_unregister(port->dev,
				       &private_data->xtal2_clk_sel_mux.hw);
		devm_clk_hw_unregister(port->dev,
				       &private_data->xtal_clk_sel_mux.hw);
		devm_clk_hw_unregister(port->dev,
				       &private_data->xtal_div2.hw);
	}

	if (!private_data->xtal_input_only)
		devm_clk_hw_unregister(port->dev, &private_data->clk81_div4.hw);

	devm_clk_hw_unregister(port->dev, &private_data->xtal_div3.hw);

	devm_iounmap(port->dev, port->membase);
	port->membase = NULL;
	devm_release_mem_region(port->dev, port->mapbase, port->mapsize);
}

static int meson_uart_register_clk(struct uart_port *port,
				   const char *name_suffix,
				   const struct clk_parent_data *parent_data,
				   unsigned int num_parents,
				   const struct clk_ops *ops,
				   struct clk_hw *hw)
{
	struct clk_init_data init = { };
	char clk_name[32];

	snprintf(clk_name, sizeof(clk_name), "%s#%s", dev_name(port->dev),
		 name_suffix);

	init.name = clk_name;
	init.ops = ops;
	init.flags = CLK_SET_RATE_PARENT;
	init.parent_data = parent_data;
	init.num_parents = num_parents;

	hw->init = &init;

	return devm_clk_hw_register(port->dev, hw);
}

static int meson_uart_request_port(struct uart_port *port)
{
	struct meson_uart_data *private_data = port->private_data;
	struct clk_parent_data use_xtal_mux_parents[2] = {
		{ .index = -1, },
		{ .index = -1, },
	};
	struct clk_parent_data xtal_clk_sel_mux_parents[2] = { };
	struct clk_parent_data xtal2_clk_sel_mux_parents[2] = { };
	struct clk_parent_data xtal_div_parent = { .fw_name = "xtal", };
	struct clk_parent_data clk81_div_parent = { .fw_name = "baud", };
	struct clk_parent_data baud_div_parent = { };
	int ret;

	if (!devm_request_mem_region(port->dev, port->mapbase, port->mapsize,
				     dev_name(port->dev))) {
		dev_err(port->dev, "Memory region busy\n");
		return -EBUSY;
	}

	port->membase = devm_ioremap(port->dev, port->mapbase,
					     port->mapsize);
	if (!port->membase)
		return -ENOMEM;

	ret = clk_prepare_enable(private_data->pclk);
	if (ret)
		return ret;

	private_data->xtal_div3.mult = 1;
	private_data->xtal_div3.div = 3;
	ret = meson_uart_register_clk(port, "xtal_div3", &xtal_div_parent,
				      1, &clk_fixed_factor_ops,
				      &private_data->xtal_div3.hw);
	if (ret)
		goto err_disable_pclk;

	if (!private_data->xtal_input_only) {
		private_data->clk81_div4.mult = 1;
		private_data->clk81_div4.div = 4;
		ret = meson_uart_register_clk(port, "clk81_div4",
					      &clk81_div_parent, 1,
					      &clk_fixed_factor_ops,
					      &private_data->clk81_div4.hw);
		if (ret)
			goto err_unregister_xtal_div3;

		use_xtal_mux_parents[0].hw = &private_data->clk81_div4.hw;
	}

	if (private_data->has_xtal_clk_sel) {
		private_data->xtal_div2.mult = 1;
		private_data->xtal_div2.div = 2;
		ret = meson_uart_register_clk(port, "xtal_div2",
					      &xtal_div_parent, 1,
					      &clk_fixed_factor_ops,
					      &private_data->xtal_div2.hw);
		if (ret)
			goto err_unregister_clk81_div4;

		xtal_clk_sel_mux_parents[0].hw = &private_data->xtal_div3.hw;
		xtal_clk_sel_mux_parents[1].fw_name = "xtal";

		private_data->xtal_clk_sel_mux.reg = port->membase + AML_UART_REG5;
		private_data->xtal_clk_sel_mux.mask = 0x1;
		private_data->xtal_clk_sel_mux.shift = 26;
		private_data->xtal_clk_sel_mux.flags = CLK_MUX_ROUND_CLOSEST;
		ret = meson_uart_register_clk(port, "xtal_clk_sel",
					      xtal_clk_sel_mux_parents,
					      ARRAY_SIZE(xtal_clk_sel_mux_parents),
					      &clk_mux_ops,
					      &private_data->xtal_clk_sel_mux.hw);
		if (ret)
			goto err_unregister_xtal_div2;

		xtal2_clk_sel_mux_parents[0].hw = &private_data->xtal_clk_sel_mux.hw;
		xtal2_clk_sel_mux_parents[1].hw = &private_data->xtal_div2.hw;

		private_data->xtal2_clk_sel_mux.reg = port->membase + AML_UART_REG5;
		private_data->xtal2_clk_sel_mux.mask = 0x1;
		private_data->xtal2_clk_sel_mux.shift = 27;
		private_data->xtal2_clk_sel_mux.flags = CLK_MUX_ROUND_CLOSEST;
		ret = meson_uart_register_clk(port, "xtal2_clk_sel",
					      xtal2_clk_sel_mux_parents,
					      ARRAY_SIZE(xtal2_clk_sel_mux_parents),
					      &clk_mux_ops,
					      &private_data->xtal2_clk_sel_mux.hw);
		if (ret)
			goto err_unregister_xtal_clk_sel;

		use_xtal_mux_parents[1].hw = &private_data->xtal2_clk_sel_mux.hw;
	} else {
		use_xtal_mux_parents[1].hw = &private_data->xtal_div3.hw;
	}

	private_data->use_xtal_mux.reg = port->membase + AML_UART_REG5;
	private_data->use_xtal_mux.mask = 0x1;
	private_data->use_xtal_mux.shift = 24;
	private_data->use_xtal_mux.flags = CLK_MUX_ROUND_CLOSEST;
	ret = meson_uart_register_clk(port, "use_xtal", use_xtal_mux_parents,
				      ARRAY_SIZE(use_xtal_mux_parents),
				      &clk_mux_ops,
				      &private_data->use_xtal_mux.hw);
	if (ret)
		goto err_unregister_xtal2_clk_sel;

	baud_div_parent.hw = &private_data->use_xtal_mux.hw;

	private_data->baud_div.reg = port->membase + AML_UART_REG5;
	private_data->baud_div.shift = 0;
	private_data->baud_div.width = 23;
	private_data->baud_div.flags = CLK_DIVIDER_ROUND_CLOSEST;
	ret = meson_uart_register_clk(port, "baud_div",
				      &baud_div_parent, 1,
				      &clk_divider_ops,
				      &private_data->baud_div.hw);
	if (ret)
		goto err_unregister_use_xtal_mux;

	private_data->baud_clk = devm_clk_hw_get_clk(port->dev,
						     &private_data->baud_div.hw,
						     "baud_rate");
	if (IS_ERR(private_data->baud_clk)) {
		ret = PTR_ERR(private_data->baud_clk);
		goto err_unregister_baud_div;
	}

	ret = clk_prepare_enable(private_data->baud_clk);
	if (ret)
		goto err_put_baud_clk;

	return 0;

err_put_baud_clk:
	// TODO: devm_clk_hw_put(port->dev, private_data->baud_clk);
err_unregister_baud_div:
	devm_clk_hw_unregister(port->dev, &private_data->baud_div.hw);
err_unregister_use_xtal_mux:
	devm_clk_hw_unregister(port->dev, &private_data->use_xtal_mux.hw);
err_unregister_xtal2_clk_sel:
	if (private_data->has_xtal_clk_sel)
		devm_clk_hw_unregister(port->dev,
				       &private_data->xtal2_clk_sel_mux.hw);
err_unregister_xtal_clk_sel:
	if (private_data->has_xtal_clk_sel)
		devm_clk_hw_unregister(port->dev,
				       &private_data->xtal_clk_sel_mux.hw);
err_unregister_xtal_div2:
	if (private_data->has_xtal_clk_sel)
		devm_clk_hw_unregister(port->dev,
				       &private_data->xtal_div2.hw);
err_unregister_clk81_div4:
	if (!private_data->xtal_input_only)
		devm_clk_hw_unregister(port->dev,
				       &private_data->clk81_div4.hw);
err_unregister_xtal_div3:
	devm_clk_hw_unregister(port->dev, &private_data->xtal_div3.hw);
err_disable_pclk:
	clk_disable_unprepare(private_data->pclk);
	return ret;
}

static void meson_uart_config_port(struct uart_port *port, int flags)
{
	if (flags & UART_CONFIG_TYPE) {
		port->type = PORT_MESON;
		meson_uart_request_port(port);
	}
}

#ifdef CONFIG_CONSOLE_POLL
/*
 * Console polling routines for writing and reading from the uart while
 * in an interrupt or debug context (i.e. kgdb).
 */

static int meson_uart_poll_get_char(struct uart_port *port)
{
	u32 c;
	unsigned long flags;

	spin_lock_irqsave(&port->lock, flags);

	if (readl(port->membase + AML_UART_STATUS) & AML_UART_RX_EMPTY)
		c = NO_POLL_CHAR;
	else
		c = readl(port->membase + AML_UART_RFIFO);

	spin_unlock_irqrestore(&port->lock, flags);

	return c;
}

static void meson_uart_poll_put_char(struct uart_port *port, unsigned char c)
{
	unsigned long flags;
	u32 reg;
	int ret;

	spin_lock_irqsave(&port->lock, flags);

	/* Wait until FIFO is empty or timeout */
	ret = readl_poll_timeout_atomic(port->membase + AML_UART_STATUS, reg,
					reg & AML_UART_TX_EMPTY,
					AML_UART_POLL_USEC,
					AML_UART_TIMEOUT_USEC);
	if (ret == -ETIMEDOUT) {
		dev_err(port->dev, "Timeout waiting for UART TX EMPTY\n");
		goto out;
	}

	/* Write the character */
	writel(c, port->membase + AML_UART_WFIFO);

	/* Wait until FIFO is empty or timeout */
	ret = readl_poll_timeout_atomic(port->membase + AML_UART_STATUS, reg,
					reg & AML_UART_TX_EMPTY,
					AML_UART_POLL_USEC,
					AML_UART_TIMEOUT_USEC);
	if (ret == -ETIMEDOUT)
		dev_err(port->dev, "Timeout waiting for UART TX EMPTY\n");

out:
	spin_unlock_irqrestore(&port->lock, flags);
}

#endif /* CONFIG_CONSOLE_POLL */

static const struct uart_ops meson_uart_ops = {
	.set_mctrl      = meson_uart_set_mctrl,
	.get_mctrl      = meson_uart_get_mctrl,
	.tx_empty	= meson_uart_tx_empty,
	.start_tx	= meson_uart_start_tx,
	.stop_tx	= meson_uart_stop_tx,
	.stop_rx	= meson_uart_stop_rx,
	.startup	= meson_uart_startup,
	.shutdown	= meson_uart_shutdown,
	.set_termios	= meson_uart_set_termios,
	.type		= meson_uart_type,
	.config_port	= meson_uart_config_port,
	.request_port	= meson_uart_request_port,
	.release_port	= meson_uart_release_port,
	.verify_port	= meson_uart_verify_port,
#ifdef CONFIG_CONSOLE_POLL
	.poll_get_char	= meson_uart_poll_get_char,
	.poll_put_char	= meson_uart_poll_put_char,
#endif
};

#ifdef CONFIG_SERIAL_MESON_CONSOLE
static void meson_uart_enable_tx_engine(struct uart_port *port)
{
	u32 val;

	val = readl(port->membase + AML_UART_CONTROL);
	val |= AML_UART_TX_EN;
	writel(val, port->membase + AML_UART_CONTROL);
}

static void meson_console_putchar(struct uart_port *port, int ch)
{
	if (!port->membase)
		return;

	while (readl(port->membase + AML_UART_STATUS) & AML_UART_TX_FULL)
		cpu_relax();
	writel(ch, port->membase + AML_UART_WFIFO);
}

static void meson_serial_port_write(struct uart_port *port, const char *s,
				    u_int count)
{
	unsigned long flags;
	int locked;
	u32 val, tmp;

	local_irq_save(flags);
	if (port->sysrq) {
		locked = 0;
	} else if (oops_in_progress) {
		locked = spin_trylock(&port->lock);
	} else {
		spin_lock(&port->lock);
		locked = 1;
	}

	val = readl(port->membase + AML_UART_CONTROL);
	tmp = val & ~(AML_UART_TX_INT_EN | AML_UART_RX_INT_EN);
	writel(tmp, port->membase + AML_UART_CONTROL);

	uart_console_write(port, s, count, meson_console_putchar);
	writel(val, port->membase + AML_UART_CONTROL);

	if (locked)
		spin_unlock(&port->lock);
	local_irq_restore(flags);
}

static void meson_serial_console_write(struct console *co, const char *s,
				       u_int count)
{
	struct uart_port *port;

	port = meson_ports[co->index];
	if (!port)
		return;

	meson_serial_port_write(port, s, count);
}

static int meson_serial_console_setup(struct console *co, char *options)
{
	struct uart_port *port;
	int baud = 115200;
	int bits = 8;
	int parity = 'n';
	int flow = 'n';

	if (co->index < 0 || co->index >= AML_UART_PORT_NUM)
		return -EINVAL;

	port = meson_ports[co->index];
	if (!port || !port->membase)
		return -ENODEV;

	meson_uart_enable_tx_engine(port);

	if (options)
		uart_parse_options(options, &baud, &parity, &bits, &flow);

	return uart_set_options(port, co, baud, parity, bits, flow);
}

static struct console meson_serial_console = {
	.name		= AML_UART_DEV_NAME,
	.write		= meson_serial_console_write,
	.device		= uart_console_device,
	.setup		= meson_serial_console_setup,
	.flags		= CON_PRINTBUFFER,
	.index		= -1,
	.data		= &meson_uart_driver,
};

static int __init meson_serial_console_init(void)
{
	register_console(&meson_serial_console);
	return 0;
}

static void meson_serial_early_console_write(struct console *co,
					     const char *s,
					     u_int count)
{
	struct earlycon_device *dev = co->data;

	meson_serial_port_write(&dev->port, s, count);
}

static int __init
meson_serial_early_console_setup(struct earlycon_device *device, const char *opt)
{
	if (!device->port.membase)
		return -ENODEV;

	meson_uart_enable_tx_engine(&device->port);
	device->con->write = meson_serial_early_console_write;
	return 0;
}

OF_EARLYCON_DECLARE(meson, "amlogic,meson-ao-uart",
		    meson_serial_early_console_setup);

#define MESON_SERIAL_CONSOLE	(&meson_serial_console)
#else
static int __init meson_serial_console_init(void) {
	return 0;
}
#define MESON_SERIAL_CONSOLE	NULL
#endif

static struct uart_driver meson_uart_driver = {
	.owner		= THIS_MODULE,
	.driver_name	= "meson_uart",
	.dev_name	= AML_UART_DEV_NAME,
	.nr		= AML_UART_PORT_NUM,
	.cons		= MESON_SERIAL_CONSOLE,
};

static int meson_uart_probe(struct platform_device *pdev)
{
	struct meson_uart_data *private_data;
	struct resource *res_mem, *res_irq;
	struct clk *clk_baud, *clk_xtal;
	struct uart_port *port;
	u32 fifosize = 64; /* Default is 64, 128 for EE UART_0 */
	int ret = 0;

	if (pdev->dev.of_node)
		pdev->id = of_alias_get_id(pdev->dev.of_node, "serial");

	if (pdev->id < 0) {
		int id;

		for (id = AML_UART_PORT_OFFSET; id < AML_UART_PORT_NUM; id++) {
			if (!meson_ports[id]) {
				pdev->id = id;
				break;
			}
		}
	}

	if (pdev->id < 0 || pdev->id >= AML_UART_PORT_NUM)
		return -EINVAL;

	res_mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res_mem)
		return -ENODEV;

	res_irq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!res_irq)
		return -ENODEV;

	of_property_read_u32(pdev->dev.of_node, "fifo-size", &fifosize);

	if (meson_ports[pdev->id]) {
		dev_err(&pdev->dev, "port %d already allocated\n", pdev->id);
		return -EBUSY;
	}

	private_data = devm_kzalloc(&pdev->dev, sizeof(*private_data),
				    GFP_KERNEL);
	if (!private_data)
		return -ENOMEM;

	private_data->pclk = devm_clk_get(&pdev->dev, "pclk");
	if (IS_ERR(private_data->pclk))
		return PTR_ERR(private_data->pclk);

	clk_baud = devm_clk_get(&pdev->dev, "baud");
	if (IS_ERR(clk_baud))
		return PTR_ERR(clk_baud);

	clk_xtal = devm_clk_get(&pdev->dev, "xtal");
	if (IS_ERR(clk_xtal))
		return PTR_ERR(clk_xtal);

	port = &private_data->port;

	port->iotype = UPIO_MEM;
	port->mapbase = res_mem->start;
	port->mapsize = resource_size(res_mem);
	port->irq = res_irq->start;
	port->flags = UPF_BOOT_AUTOCONF | UPF_LOW_LATENCY;
	port->has_sysrq = IS_ENABLED(CONFIG_SERIAL_MESON_CONSOLE);
	port->dev = &pdev->dev;
	port->line = pdev->id;
	port->type = PORT_MESON;
	port->x_char = 0;
	port->ops = &meson_uart_ops;
	port->fifosize = fifosize;
	port->uartclk = clk_get_rate(clk_baud);
	port->private_data = private_data;

	if (port->uartclk == clk_get_rate(clk_xtal))
		private_data->xtal_input_only = true;

	if (device_get_match_data(&pdev->dev))
		private_data->has_xtal_clk_sel = true;

	meson_ports[pdev->id] = port;
	platform_set_drvdata(pdev, port);

	ret = uart_add_one_port(&meson_uart_driver, port);
	if (ret)
		meson_ports[pdev->id] = NULL;

	return ret;
}

static int meson_uart_remove(struct platform_device *pdev)
{
	struct uart_port *port;

	port = platform_get_drvdata(pdev);
	uart_remove_one_port(&meson_uart_driver, port);
	meson_ports[pdev->id] = NULL;

	return 0;
}

static const struct of_device_id meson_uart_dt_match[] = {
	{
		.compatible = "amlogic,meson6-uart",
		.data = (void *) false,
	},
	{
		.compatible = "amlogic,meson8-uart",
		.data = (void *) false,
	},
	{
		.compatible = "amlogic,meson8b-uart",
		.data = (void *) false,
	},
	{
		.compatible = "amlogic,meson-gxbb-uart",
		.data = (void *) false,
	},
	{
		.compatible = "amlogic,meson-gxl-uart",
		.data = (void *) true,
	},
	{
		.compatible = "amlogic,meson-g12a-uart",
		.data = (void *) true,
	},
	/* deprecated, don't use anymore */
	{
		.compatible = "amlogic,meson-gx-uart",
		.data = (void *) false,
	},
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, meson_uart_dt_match);

static  struct platform_driver meson_uart_platform_driver = {
	.probe		= meson_uart_probe,
	.remove		= meson_uart_remove,
	.driver		= {
		.name		= "meson_uart",
		.of_match_table	= meson_uart_dt_match,
	},
};

static int __init meson_uart_init(void)
{
	int ret;

	ret = meson_serial_console_init();
	if (ret)
		return ret;
	
	ret = uart_register_driver(&meson_uart_driver);
	if (ret)
		return ret;

	ret = platform_driver_register(&meson_uart_platform_driver);
	if (ret)
		uart_unregister_driver(&meson_uart_driver);

	return ret;
}

static void __exit meson_uart_exit(void)
{
	platform_driver_unregister(&meson_uart_platform_driver);
	uart_unregister_driver(&meson_uart_driver);
}

module_init(meson_uart_init);
module_exit(meson_uart_exit);

MODULE_AUTHOR("Carlo Caione <carlo@caione.org>");
MODULE_DESCRIPTION("Amlogic Meson serial port driver");
MODULE_LICENSE("GPL v2");
