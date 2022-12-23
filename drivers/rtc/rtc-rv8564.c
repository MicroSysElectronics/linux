/* -*-C-*- */
/* SPDX-License-Identifier:    GPL-2.0+ */
/*
 * Copyright (C) 2020 MicroSys Electronics GmbH
 * Author: Kay Potthoff <kay.potthoff@microsys.de>
 *
 */

/*!
 * \addtogroup RTC Driver for Micro Crystal RV8564
 * @{
 *
 * \file
 * This file contains a Linux driver for the RV8564 from Micro Crystal.
 */

#include <linux/i2c.h>
#include <linux/bcd.h>
#include <linux/rtc.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/pm_wakeirq.h>
#include <linux/regmap.h>
#include <linux/of_platform.h>
#include <linux/of_irq.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>

struct regmap_config rv8564_regmap = {
        .reg_bits = 8,
        .val_bits = 8,
        .max_register = 0x10,
};

typedef struct rv8564 {
    struct rtc_device   *rtc;
    struct regmap       *regmap;
    struct work_struct irq_work;
    int irq;
} rv8564_t;

static int rv8564_stop_bit(struct device *dev, bool stop)
{
    rv8564_t *rv8564 = dev_get_drvdata(dev);
    uint8_t buf[1] = { 0x00 };
    int rv = 0;

    rv = regmap_bulk_read(rv8564->regmap, 0x00, buf, sizeof(buf));;

    if (rv != 0) return rv;

    if (stop)
        buf[0] |= BIT(5);
    else
        buf[0] &= ~BIT(5);

    rv = regmap_bulk_write(rv8564->regmap, 0x00, buf, sizeof(buf));

    return rv;
}

static int rv8564_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
    rv8564_t *rv8564 = dev_get_drvdata(dev);
    int rv = 0;
    uint8_t buf[7];

    rv = regmap_bulk_read(rv8564->regmap, 0x02, buf, sizeof(buf));

    if (rv)
        return rv;

    tm->tm_sec  = bcd2bin(buf[0] & 0x7F);
    tm->tm_min  = bcd2bin(buf[1] & 0x7F);
    tm->tm_hour = bcd2bin(buf[2] & 0x3F);

    tm->tm_mday = bcd2bin(buf[3] & 0x3F);
    tm->tm_wday = buf[4] & 0x07;
    tm->tm_mon  = bcd2bin(buf[5] & 0x1F) - 1;
    tm->tm_year = bcd2bin(buf[6]) + 100;

    return rv;
}

static int rv8564_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
    rv8564_t *rv8564 = dev_get_drvdata(dev);
    int rv = 0;
    uint8_t buf[7];

    buf[0] = bin2bcd(tm->tm_sec) & 0x7F;
    buf[1] = bin2bcd(tm->tm_min) & 0x7f;
    buf[2] = bin2bcd(tm->tm_hour) & 0x3f;

    buf[3] = bin2bcd(tm->tm_mday);
    buf[5] = bin2bcd(tm->tm_mon + 1) & 0x1f;
    buf[6] = bin2bcd(tm->tm_year - 100);

    buf[4] = tm->tm_wday & 0x07;

    rv8564_stop_bit(dev, true);

    rv = regmap_bulk_write(rv8564->regmap, 0x02, buf, sizeof(buf));

    rv8564_stop_bit(dev, false);

    return rv;
}

static const struct rtc_class_ops rv8564_rtc_ops = {
    .read_time  = rv8564_rtc_read_time,
    .set_time   = rv8564_rtc_set_time,
};

static int rv8564_probe(struct i2c_client *client)
{
    rv8564_t *rv8564;

    rv8564 = devm_kzalloc(&client->dev, sizeof(struct rv8564), GFP_KERNEL);

    if (!rv8564)
        return -ENOMEM;

    rv8564->regmap = devm_regmap_init_i2c(client, &rv8564_regmap);

    if (IS_ERR(rv8564->regmap))
        return PTR_ERR(rv8564->regmap);

    i2c_set_clientdata(client, rv8564);

    rv8564->rtc = devm_rtc_allocate_device(&client->dev);

    if (IS_ERR(rv8564->rtc))
        return PTR_ERR(rv8564->rtc);

    rv8564->rtc->ops = &rv8564_rtc_ops;
    rv8564->rtc->range_min = RTC_TIMESTAMP_BEGIN_2000;
    rv8564->rtc->range_max = RTC_TIMESTAMP_END_2099;
    rv8564->rtc->uie_unsupported = 1;

    return rtc_register_device(rv8564->rtc);
}

static const struct of_device_id rv8564_of_match[] = {
    { .compatible = "microcrystal,rv8564" },
    {}
};
MODULE_DEVICE_TABLE(of, rv8564_of_match);

static struct i2c_driver rv8564_driver = {
    .driver     = {
        .name   = "rtc-rv8564",
        .of_match_table = of_match_ptr(rv8564_of_match),
    },
    .probe_new  = rv8564_probe,
};

module_i2c_driver(rv8564_driver);

MODULE_AUTHOR("Kay Potthoff <kay.potthoff@microsys.de>");
MODULE_DESCRIPTION("RV8564 RTC driver");
MODULE_LICENSE("GPL");

/*!@}*/

/* *INDENT-OFF* */
/******************************************************************************
 * Local Variables:
 * mode: C
 * c-indent-level: 4
 * c-basic-offset: 4
 * tab-width: 4
 * indent-tabs-mode: nil
 * End:
 * kate: space-indent on; indent-width 4; mixedindent off; indent-mode cstyle;
 * vim: set expandtab filetype=c:
 * vi: set et tabstop=4 shiftwidth=4: */
