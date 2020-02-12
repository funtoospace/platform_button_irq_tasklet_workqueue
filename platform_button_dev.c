#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>

static struct platform_device *platform_button_pdev;




static int __init platform_button_dev_init(void)
{
	int ret;

	platform_button_pdev = platform_device_alloc("platform_button", -1);
	if (!platform_button_pdev)
		return -ENOMEM;

	ret = platform_device_add(platform_button_pdev);
	if (ret) {
		platform_device_put(platform_button_pdev);
		return ret;
	}


	printk(KERN_INFO "platform_button is created !!!\n");
	
	return 0;

}
module_init(platform_button_dev_init);

static void __exit platform_button_dev_exit(void)
{
	platform_device_unregister(platform_button_pdev);
}
module_exit(platform_button_dev_exit);

MODULE_AUTHOR("WL");
MODULE_LICENSE("GPL");
