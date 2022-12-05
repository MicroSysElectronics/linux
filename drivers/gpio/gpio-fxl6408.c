/*
 *  Copyright (C) 2016 Broadcom Limited.
 *  Copyright (C) 2019-2022 MicroSys Electronics GmbH <kay.potthoff@microsys.de>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; version 2 of the License.
 */

/**
 * DOC: FXL6408 I2C to GPIO expander.
 *
 * This chip has has 8 GPIO lines out of it, and is controlled by an
 * I2C bus (a pair of lines), providing 4x expansion of GPIO lines.
 * It also provides an interrupt line out for notifying of
 * statechanges.
 *
 * Any preconfigured state will be left in place until the GPIO lines
 * get activated.  At power on, everything is treated as an input.
 *
 * Documentation can be found at:
 * https://www.fairchildsemi.com/datasheets/FX/FXL6408.pdf
 */

/**
 *
 * MicroSys Electronics GmbH
 * =========================
 *
 * Added interrupt controller capability.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/of_platform.h>
#include <linux/of_irq.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/spinlock.h>

#define FXL6408_DEVICE_ID	0x01
#define FXL6408_RST_INT		BIT(1)
#define FXL6408_SW_RST		BIT(0)

/* Bits set here indicate that the GPIO is an output. */
#define FXL6408_IO_DIR 0x03
/* Bits set here, when the corresponding bit of IO_DIR is set, drive
 * the output high instead of low.
 */
#define FXL6408_OUTPUT 0x05
/* Bits here make the output High-Z, instead of the OUTPUT value. */
#define FXL6408_OUTPUT_HIGH_Z 0x07
/* Bits here define the expected input state of the GPIO.
 * INTERRUPT_STAT bits will be set when the INPUT transitions away
 * from this value.
 */
#define FXL6408_INPUT_DEFAULT_STATE 0x09
/* Bits here enable either pull up or pull down according to
 * FXL6408_PULL_DOWN.
 */
#define FXL6408_PULL_ENABLE 0x0b
/* Bits set here (when the corresponding PULL_ENABLE is set) enable a
 * pull-up instead of a pull-down.
 */
#define FXL6408_PULL_UP 0x0d
/* Returns the current status (1 = HIGH) of the input pins. */
#define FXL6408_INPUT_STATUS 0x0f
/* Mask of pins which can generate interrupts. */
#define FXL6408_INTERRUPT_MASK 0x11
/* Mask of pins which have generated an interrupt.  Cleared on read. */
#define FXL6408_INTERRUPT_STAT 0x13

#define FXL6408_MAX_NGPIOS (8)

struct fxl6408_chip {
	struct work_struct irq_work;
	struct work_struct set_mask_work;
	struct gpio_chip gpio_chip;
	struct irq_chip irq_chip;
	struct i2c_client *client;
	struct mutex i2c_lock;
	struct mutex irq_lock;
	raw_spinlock_t mask_lock;

	/* Caches of register values so we don't have to read-modify-write. */
	u8 reg_io_dir;
	u8 reg_output;

	u8 irq_mask;
};

static int fxl6408_gpio_direction_input(struct gpio_chip *gc, unsigned off)
{
	struct fxl6408_chip *chip = gpiochip_get_data(gc);

	mutex_lock(&chip->i2c_lock);
	chip->reg_io_dir &= ~BIT(off);
	i2c_smbus_write_byte_data(chip->client, FXL6408_IO_DIR,
			chip->reg_io_dir);
	mutex_unlock(&chip->i2c_lock);

	return 0;
}

static int fxl6408_gpio_direction_output(struct gpio_chip *gc, unsigned off,
		int val)
{
	struct fxl6408_chip *chip = gpiochip_get_data(gc);

	mutex_lock(&chip->i2c_lock);
	chip->reg_io_dir |= BIT(off);
	i2c_smbus_write_byte_data(chip->client, FXL6408_IO_DIR,
			chip->reg_io_dir);
	mutex_unlock(&chip->i2c_lock);

	return 0;
}

static int fxl6408_gpio_get_direction(struct gpio_chip *gc, unsigned off)
{
	struct fxl6408_chip *chip = gpiochip_get_data(gc);

	return (chip->reg_io_dir & BIT(off)) == 0;
}

static int fxl6408_gpio_get_value(struct gpio_chip *gc, unsigned off)
{
	struct fxl6408_chip *chip = gpiochip_get_data(gc);
	u8 reg;

	const int dir = fxl6408_gpio_get_direction(gc, off);

	mutex_lock(&chip->i2c_lock);
	if (dir)
		reg = i2c_smbus_read_byte_data(chip->client,
				FXL6408_INPUT_STATUS);
	else
		reg = chip->reg_output;
	mutex_unlock(&chip->i2c_lock);

	return (reg & BIT(off)) != 0;
}

static void fxl6408_gpio_set_value(struct gpio_chip *gc, unsigned off, int val)
{
	struct fxl6408_chip *chip = gpiochip_get_data(gc);

	mutex_lock(&chip->i2c_lock);

	if (val)
		chip->reg_output |= BIT(off);
	else
		chip->reg_output &= ~BIT(off);

	i2c_smbus_write_byte_data(chip->client, FXL6408_OUTPUT,
			chip->reg_output);
	mutex_unlock(&chip->i2c_lock);
}

static void fxl6408_gpio_set_multiple(struct gpio_chip *gc, unsigned long *mask,
		unsigned long *bits)
{
	struct fxl6408_chip *chip = gpiochip_get_data(gc);

	mutex_lock(&chip->i2c_lock);
	chip->reg_output = (chip->reg_output & ~mask[0]) | bits[0];
	i2c_smbus_write_byte_data(chip->client, FXL6408_OUTPUT,
			chip->reg_output);
	mutex_unlock(&chip->i2c_lock);
}

static int fxl6408_set_config(struct gpio_chip *gc,
		unsigned int offset,
		unsigned long config)
{
	struct fxl6408_chip *chip = gpiochip_get_data(gc);
	u8 pull_up, pull_en;

	mutex_lock(&chip->i2c_lock);

	pull_up = i2c_smbus_read_byte_data(chip->client, FXL6408_PULL_UP);
	pull_en = i2c_smbus_read_byte_data(chip->client, FXL6408_PULL_ENABLE);

	switch (pinconf_to_config_param(config)) {
	case PIN_CONFIG_DRIVE_OPEN_DRAIN:
		// drive the line low or go high impedance
		pull_en &= ~BIT(offset);
		pull_up &= ~BIT(offset);
		break;
	case PIN_CONFIG_DRIVE_PUSH_PULL:
		// drive the line both high and low
		pull_en |= BIT(offset);
		break;
	case PIN_CONFIG_DRIVE_OPEN_SOURCE:
		// drive the line high or go high impedance
		pull_en &= ~BIT(offset);
		pull_up |= BIT(offset);
		break;
	default:
		break;
	}

	i2c_smbus_write_byte_data(chip->client, FXL6408_PULL_UP, pull_up);
	i2c_smbus_write_byte_data(chip->client, FXL6408_PULL_ENABLE, pull_en);

	mutex_unlock(&chip->i2c_lock);

	return 0;
}

static void fxl6408_irq_clear(struct fxl6408_chip *chip)
{
	mutex_lock(&chip->i2c_lock);
	// Clear IRQs:
	i2c_smbus_read_byte_data(chip->client, FXL6408_INTERRUPT_STAT);
	mutex_unlock(&chip->i2c_lock);
}

static void fxl6408_set_mask_worker(struct work_struct *work)
{
	struct fxl6408_chip *chip;

	chip = container_of(work, struct fxl6408_chip, set_mask_work);

	mutex_lock(&chip->i2c_lock);
	i2c_smbus_write_byte_data(chip->client, FXL6408_INTERRUPT_MASK,
			chip->irq_mask);
	mutex_unlock(&chip->i2c_lock);
}

static void fxl6408_irq_mask(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct fxl6408_chip *chip = gpiochip_get_data(gc);
	unsigned long flags;

	raw_spin_lock_irqsave(&chip->mask_lock, flags);
	chip->irq_mask |= BIT(d->hwirq);
	raw_spin_unlock_irqrestore(&chip->mask_lock, flags);

	schedule_work(&chip->set_mask_work);
}

static void fxl6408_irq_unmask(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct fxl6408_chip *chip = gpiochip_get_data(gc);
	unsigned long flags;

	raw_spin_lock_irqsave(&chip->mask_lock, flags);
	chip->irq_mask &= ~BIT(d->hwirq);
	raw_spin_unlock_irqrestore(&chip->mask_lock, flags);

	schedule_work(&chip->set_mask_work);
}

static void fxl6408_irq_bus_lock(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct fxl6408_chip *chip = gpiochip_get_data(gc);

	mutex_lock(&chip->irq_lock);
}

static void fxl6408_irq_bus_sync_unlock(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct fxl6408_chip *chip = gpiochip_get_data(gc);

	mutex_unlock(&chip->irq_lock);
}

static int fxl6408_irq_set_type(struct irq_data *d, unsigned int type)
{
	const unsigned int irq_type = type & IRQ_TYPE_SENSE_MASK;

	return (irq_type & IRQ_TYPE_LEVEL_LOW)
		|| (irq_type & IRQ_TYPE_EDGE_FALLING);
}

static void fxl6408_worker(struct work_struct *work)
{
	struct fxl6408_chip *chip;
	u8 irq_status, irq_mask;
	int i, irq_mapping;
	unsigned long flags;

	chip = container_of(work, struct fxl6408_chip, irq_work);

	mutex_lock(&chip->i2c_lock);
	// read and clear IRQ status:
	irq_status = i2c_smbus_read_byte_data(chip->client,
			FXL6408_INTERRUPT_STAT);
	mutex_unlock(&chip->i2c_lock);

	if (irq_status == 0) {
		enable_irq(chip->client->irq);
		return;
	}

	raw_spin_lock_irqsave(&chip->mask_lock, flags);
	irq_mask = chip->irq_mask;
	raw_spin_unlock_irqrestore(&chip->mask_lock, flags);

	for (i = 0; i < chip->gpio_chip.ngpio; i++) {
		if ((irq_mask & BIT(i))==0
				&& (irq_status & BIT(i))) {
			irq_mapping = irq_find_mapping(
					chip->gpio_chip.irq.domain, i);
			handle_nested_irq(irq_mapping);
		}
	}

	enable_irq(chip->client->irq);
}

static irqreturn_t fxl6408_irq_handler(int irq, void *devid)
{
	struct fxl6408_chip *chip = devid;

	if (chip->irq_mask == ~0) return IRQ_NONE;

	disable_irq_nosync(chip->client->irq);

	schedule_work(&chip->irq_work);

	return IRQ_HANDLED;
}

static void fxl6408_irq_disable(struct fxl6408_chip *chip)
{
	mutex_lock(&chip->i2c_lock);
	// Mask all IRQs:
	chip->irq_mask = ~0;
	i2c_smbus_write_byte_data(chip->client, FXL6408_INTERRUPT_MASK,
			chip->irq_mask);
	mutex_unlock(&chip->i2c_lock);

	// Clear IRQs:
	fxl6408_irq_clear(chip);
}

static int fxl6408_irq_setup(struct fxl6408_chip *chip, int irq_base)
{
	struct i2c_client *client = chip->client;
	int ret;

	fxl6408_irq_disable(chip);

	if (client->irq && irq_base != -1) {

		chip->irq_chip.name = "fxl6408";
		chip->irq_chip.irq_mask = fxl6408_irq_mask;
		chip->irq_chip.irq_unmask = fxl6408_irq_unmask;
		chip->irq_chip.irq_bus_lock = fxl6408_irq_bus_lock;
		chip->irq_chip.irq_bus_sync_unlock
			= fxl6408_irq_bus_sync_unlock;
		chip->irq_chip.irq_set_type = fxl6408_irq_set_type;

		raw_spin_lock_init(&chip->mask_lock);
		mutex_init(&chip->irq_lock);

		INIT_WORK(&(chip->irq_work), fxl6408_worker);
		INIT_WORK(&(chip->set_mask_work), fxl6408_set_mask_worker);

		ret = devm_request_threaded_irq(&client->dev,
				client->irq,
				NULL,
				fxl6408_irq_handler,
				IRQF_TRIGGER_PROBE | IRQF_ONESHOT |
				IRQF_SHARED,
				client->name, chip);

		if (ret) {
			dev_err(&client->dev, "failed to request irq %d\n",
					client->irq);
			return ret;
		}

		ret =  gpiochip_irqchip_add_nested(&chip->gpio_chip,
				&chip->irq_chip,
				irq_base,
				handle_edge_irq,
				IRQ_TYPE_NONE);

		if (ret) {
			dev_err(&client->dev,
				"could not connect irqchip to gpiochip\n");
			return ret;
		}

		gpiochip_set_nested_irqchip(&chip->gpio_chip,
				&chip->irq_chip,
				client->irq);
	}

	return 0;
}

static int fxl6408_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct fxl6408_chip *chip;
	struct gpio_chip *gc;
	int ret;
	u8 device_id;
	int irq_base = 0;
	struct device_node *np;
	u8 pull_cfg[2];
	u8 state;
	uint32_t ngpios = FXL6408_MAX_NGPIOS;

	dev_info(dev, "initializing\n");

	/* Check the device ID register to see if it's responding.
	 * This also clears RST_INT as a side effect, so we won't get
	 * the "we've been power cycled" interrupt once we enable
	 * interrupts.
	 */
	device_id = i2c_smbus_read_byte_data(client, FXL6408_DEVICE_ID);
	if (device_id < 0) {
		dev_err(dev, "FXL6408 probe returned %d\n", device_id);
		return device_id;
	} else if (device_id >> 5 != 5) {
		dev_err(dev, "FXL6408 probe returned DID: 0x%02x\n", device_id);
		return -ENODEV;
	}

	/* Disable High-Z of outputs, so that our OUTPUT updates
	 * actually take effect.
	 */
	i2c_smbus_write_byte_data(client, FXL6408_OUTPUT_HIGH_Z, 0);

	chip = devm_kzalloc(dev, sizeof(struct fxl6408_chip), GFP_KERNEL);
	if (chip == NULL)
		return -ENOMEM;

	chip->client = client;
	mutex_init(&chip->i2c_lock);

	chip->reg_output = i2c_smbus_read_byte_data(client, FXL6408_OUTPUT);

	snprintf(client->name, I2C_NAME_SIZE, "fxl6408-%d:%02x",
			client->adapter->nr,
			client->addr);

	gc = &chip->gpio_chip;
	gc->direction_input = fxl6408_gpio_direction_input;
	gc->direction_output = fxl6408_gpio_direction_output;
	gc->get_direction = fxl6408_gpio_get_direction;
	gc->get = fxl6408_gpio_get_value;
	gc->set = fxl6408_gpio_set_value;
	gc->set_multiple = fxl6408_gpio_set_multiple;
	gc->set_config = fxl6408_set_config;
	gc->can_sleep = true;

	gc->base = -1;

	ret = device_property_read_u32(dev, "ngpios", &ngpios);
	gc->ngpio = (uint16_t) ngpios;
	if (ret || (ngpios > FXL6408_MAX_NGPIOS))
		gc->ngpio = FXL6408_MAX_NGPIOS;

	gc->label = chip->client->name;
	gc->parent = dev;
	gc->owner = THIS_MODULE;

	i2c_set_clientdata(client, chip);

	np = client->dev.of_node;

	if (of_property_read_u8_array(np, "pull-config", pull_cfg, 2)==0) {
		i2c_smbus_write_byte_data(client, FXL6408_PULL_UP, pull_cfg[1]);
		i2c_smbus_write_byte_data(client, FXL6408_PULL_ENABLE,
				pull_cfg[0]);
	}

	if (of_property_read_u8(np, "output-default-state", &state)==0) {
		i2c_smbus_write_byte_data(client, FXL6408_OUTPUT, state);
		chip->reg_output = state;
	}

	if (of_property_read_u8(np, "input-default-state", &state)==0) {
		i2c_smbus_write_byte_data(client, FXL6408_INPUT_DEFAULT_STATE,
				state);
	}

	if (of_property_read_u8(np, "direction", &state)==0) {
		i2c_smbus_write_byte_data(client, FXL6408_IO_DIR, state);
		chip->reg_io_dir = state;
	}
	else
		chip->reg_io_dir = i2c_smbus_read_byte_data(client,
				FXL6408_IO_DIR);

	/*
	 * Register the GPIO chip after default states are set.
	 * So GPIO hogs can override the defaults.
	 */
	ret = gpiochip_add_data(gc, chip);
	if (ret)
		return ret;

	ret = of_irq_get(np, 0);

//	if (ret < 0)
//		dev_info(dev, "No interrupt resource from OF: %d\n", ret);

	if (ret == -EPROBE_DEFER) {
		return ret;
	}

	if (ret > 0) {
		client->irq = ret;
		ret = fxl6408_irq_setup(chip, irq_base);
		if (ret)
			return ret;
	}

	return 0;
}

static int fxl6408_remove(struct i2c_client *client)
{
	struct fxl6408_chip *chip = i2c_get_clientdata(client);

	gpiochip_remove(&chip->gpio_chip);

	return 0;
}

static const struct of_device_id fxl6408_dt_ids[] = {
		{.compatible = "fcs,fxl6408"},
		{}
};

MODULE_DEVICE_TABLE(of, fxl6408_dt_ids);

static const struct i2c_device_id fxl6408_id[] = {
		{"fxl6408", 0},
		{},
};

MODULE_DEVICE_TABLE(i2c, fxl6408_id);

static struct i2c_driver fxl6408_driver = {
		.driver = {
				.name = "fxl6408",
				.of_match_table = fxl6408_dt_ids,
		},
		.probe = fxl6408_probe,
		.remove = fxl6408_remove,
		.id_table = fxl6408_id,
};

module_i2c_driver(fxl6408_driver);

MODULE_AUTHOR("Eric Anholt <eric@anholt.net>");
MODULE_DESCRIPTION("GPIO expander driver for FXL6408");
MODULE_LICENSE("GPL");
