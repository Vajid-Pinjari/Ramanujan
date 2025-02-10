#include <linux/module.h>
#define INCLUDE_VERMAGIC
#include <linux/build-salt.h>
#include <linux/elfnote-lto.h>
#include <linux/export-internal.h>
#include <linux/vermagic.h>
#include <linux/compiler.h>

#ifdef CONFIG_UNWINDER_ORC
#include <asm/orc_header.h>
ORC_HEADER;
#endif

BUILD_SALT;
BUILD_LTO_INFO;

MODULE_INFO(vermagic, VERMAGIC_STRING);
MODULE_INFO(name, KBUILD_MODNAME);

__visible struct module __this_module
__section(".gnu.linkonce.this_module") = {
	.name = KBUILD_MODNAME,
	.init = init_module,
#ifdef CONFIG_MODULE_UNLOAD
	.exit = cleanup_module,
#endif
	.arch = MODULE_ARCH_INIT,
};

#ifdef CONFIG_RETPOLINE
MODULE_INFO(retpoline, "Y");
#endif



static const char ____versions[]
__used __section("__versions") =
	"\x20\x00\x00\x00\xd4\xd5\xa8\xc4"
	"clk_hw_register_clkdev\0\0"
	"\x1c\x00\x00\x00\x13\x4a\x1e\x8e"
	"devm_clk_register\0\0\0"
	"\x14\x00\x00\x00\x6c\xc6\xe4\xbf"
	"_dev_warn\0\0\0"
	"\x20\x00\x00\x00\x00\x48\x60\xc5"
	"clk_set_rate_exclusive\0\0"
	"\x14\x00\x00\x00\x71\x73\x9a\x7c"
	"clk_prepare\0"
	"\x14\x00\x00\x00\x10\xf5\xd0\x5b"
	"_dev_err\0\0\0\0"
	"\x14\x00\x00\x00\x9d\xd9\xe6\xb6"
	"clk_disable\0"
	"\x18\x00\x00\x00\x0a\xe7\x77\xb0"
	"clk_unprepare\0\0\0"
	"\x20\x00\x00\x00\x8c\xd8\xb4\xac"
	"clk_rate_exclusive_put\0\0"
	"\x14\x00\x00\x00\xa6\x88\x55\x81"
	"clk_enable\0\0"
	"\x1c\x00\x00\x00\x0c\xaf\xed\x68"
	"platform_get_irq\0\0\0\0"
	"\x20\x00\x00\x00\x8e\x83\xd5\x92"
	"request_threaded_irq\0\0\0\0"
	"\x18\x00\x00\x00\x87\x9c\x0e\xbd"
	"i2c_add_adapter\0"
	"\x14\x00\x00\x00\x3b\x4a\x51\xc1"
	"free_irq\0\0\0\0"
	"\x18\x00\x00\x00\x8d\xd0\xef\xb9"
	"dev_err_probe\0\0\0"
	"\x1c\x00\x00\x00\xcb\xf6\xfd\xf0"
	"__stack_chk_fail\0\0\0\0"
	"\x24\x00\x00\x00\xda\x90\x30\x27"
	"platform_driver_unregister\0\0"
	"\x24\x00\x00\x00\x0e\xd7\x3a\x4a"
	"wait_for_completion_timeout\0"
	"\x1c\x00\x00\x00\x46\x5f\x4c\xa1"
	"__dynamic_dev_dbg\0\0\0"
	"\x2c\x00\x00\x00\xc6\xfa\xb1\x54"
	"__ubsan_handle_load_invalid_value\0\0\0"
	"\x14\x00\x00\x00\x2f\x7a\x25\xa6"
	"complete\0\0\0\0"
	"\x14\x00\x00\x00\xbb\x6d\xfb\xbd"
	"__fentry__\0\0"
	"\x1c\x00\x00\x00\xca\x39\x82\x5b"
	"__x86_return_thunk\0\0"
	"\x24\x00\x00\x00\x5d\x84\xb4\x01"
	"__platform_driver_register\0\0"
	"\x18\x00\x00\x00\x5e\xd0\xc5\xfe"
	"devm_kmalloc\0\0\0\0"
	"\x20\x00\x00\x00\xb5\x41\x87\x60"
	"__init_swait_queue_head\0"
	"\x30\x00\x00\x00\xe6\x87\x67\x7a"
	"devm_platform_get_and_ioremap_resource\0\0"
	"\x18\x00\x00\x00\x2c\xfa\xc5\xe3"
	"devm_clk_get\0\0\0\0"
	"\x14\x00\x00\x00\x6e\x4a\x6e\x65"
	"snprintf\0\0\0\0"
	"\x18\x00\x00\x00\xce\xd8\x69\xc5"
	"__clk_get_name\0\0"
	"\x18\x00\x00\x00\xd7\xd3\x75\x6d"
	"module_layout\0\0\0"
	"\x00\x00\x00\x00\x00\x00\x00\x00";

MODULE_INFO(depends, "");

MODULE_ALIAS("of:Nmfr_i2cTi2c*Cmfr_i2c0");
MODULE_ALIAS("of:Nmfr_i2cTi2c*Cmfr_i2c0C*");
MODULE_ALIAS("of:N*T*Cmfr_i2c1");
MODULE_ALIAS("of:N*T*Cmfr_i2c1C*");

MODULE_INFO(srcversion, "68D2ED96366B192519893CB");
