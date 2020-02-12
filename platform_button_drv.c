#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/sched.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/poll.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <asm/irq.h>
#include <asm/io.h>
#include <asm/arch/regs-gpio.h>
#include <asm/hardware.h>
#include <linux/workqueue.h>


//volatile unsigned long *gpfcon = NULL;
//volatile unsigned long *gpfdat = NULL;
//volatile unsigned long *gpgcon = NULL;
//volatile unsigned long *gpgdat = NULL;

struct tasklet_struct newtask;
struct workqueue_struct *myworkqueue;
struct work_struct newwork;


static DECLARE_WAIT_QUEUE_HEAD(button_waitq);

static volatile int ev_press = 0;

struct pin_desc {
	unsigned int pin;
	unsigned int key_val;
};

/* button up  : 0x01, 0x02, 0x03, 0x04 */
/* button down: 0x81, 0x82, 0x83, 0x84 */
static unsigned char key_state;

struct pin_desc pins_desc[2] = {
	//{S3C2410_GPF0, 0x01},
	//{S3C2410_GPF2, 0x02},
	{S3C2410_GPG3, 0X03},
	{S3C2410_GPG11, 0X04},
};


struct platform_button_dev {
	struct cdev cdev;
	unsigned int flag;
	struct mutex mutex;
	wait_queue_head_t r_wait;
//	wait_queue_head_t w_wait;
//	struct fasync_struct *async_queue;
	struct miscdevice miscdev;
};

struct platform_button_dev *pld;

static void newtask_func(unsigned long data)
{
	if(in_interrupt()) {
		printk(KERN_INFO "%s in interrupt handle!\n", __func__);
	}
}

static void newwork_func(struct work_struct *work)
{
	if(in_interrupt()) {
		printk(KERN_INFO "%s in interrupt handle!\n", __func__);
	}
	msleep(2);
	printk(KERN_INFO "%s in process handle!\n", __func__);
}

static irqreturn_t buttons_irq(int irq, void *dev_id)
{
	tasklet_schedule(&newtask);

	struct pin_desc * pindesc = (struct pin_desc *)dev_id;
	unsigned int pinval;

	pinval = s3c2410_gpio_getpin(pindesc->pin);
	
	schedule_work(&newwork);
	if(in_interrupt()) {
		printk(KERN_INFO "%s in interrupt handle!\n", __func__);
	}

	if(pinval)
	{
		key_state = 0x80 | pindesc->key_val;
		//*gpfdat &= ~((1<<4) | (1<<5) | (1<<6));
		s3c2410_gpio_setpin(S3C2410_GPF4, 0);
		s3c2410_gpio_setpin(S3C2410_GPF5, 0);
		s3c2410_gpio_setpin(S3C2410_GPF6, 0);
	}
	else
	{
		key_state = pindesc->key_val;
		//*gpfdat |= (1<<4) | (1<<5) | (1<<6);
		s3c2410_gpio_setpin(S3C2410_GPF4, 1);
		s3c2410_gpio_setpin(S3C2410_GPF5, 1);
		s3c2410_gpio_setpin(S3C2410_GPF6, 1);
	}

	ev_press = 1;
	wake_up_interruptible(&button_waitq);


	return IRQ_RETVAL(IRQ_HANDLED);
}

static int platform_button_open(struct inode *inode, struct file *filp)
{
	int newdata = 100;

	filp->private_data = pld;

	printk("platform_button_open\n");

	tasklet_init(&newtask, newtask_func, newdata);

	myworkqueue = create_workqueue("my work queue");
	INIT_WORK(&newwork, newwork_func);
	queue_work(myworkqueue, &newwork);



	/* 配置GPF4,5,6为输出 中文 */
	//*gpfcon &= ~((0x3<<(4*2)) | (0x3<<(5*2)) | (0x3<<(6*2)));
	//*gpfcon |= ((0x1<<(4*2)) | (0x1<<(5*2)) | (0x1<<(6*2)));
	s3c2410_gpio_cfgpin(S3C2410_GPF4, S3C2410_GPF4_OUTP);
	s3c2410_gpio_cfgpin(S3C2410_GPF5, S3C2410_GPF5_OUTP);
	s3c2410_gpio_cfgpin(S3C2410_GPF6, S3C2410_GPF6_OUTP);
	//request_irq(IRQ_EINT0,  buttons_irq, IRQT_BOTHEDGE, "S2", &pins_desc[0]);
	//request_irq(IRQ_EINT2,  buttons_irq, IRQT_BOTHEDGE, "S3", &pins_desc[1]);
	request_irq(IRQ_EINT11, buttons_irq, IRQT_BOTHEDGE, "S4", &pins_desc[0]);
	request_irq(IRQ_EINT19, buttons_irq, IRQT_BOTHEDGE, "S5", &pins_desc[1]);
	printk("platform_button_open ok\n");
	return 0;
}

static int platform_button_poll(struct file *filp, poll_table * wait)
{
	unsigned int mask =0;

	struct platform_button_dev *dev = filp->private_data;

	mutex_lock(&dev->mutex);

	poll_wait(filp, &dev->r_wait, wait);

	if (dev->flag == 1) {
		mask |= POLLIN | POLLRDNORM;
		printk(KERN_INFO "mask is : %d\n", mask);
	}
	
	if (dev->flag == 0) {
		mask &= ~(POLL_IN | POLLRDNORM);
		printk(KERN_INFO "mask is : %d\n", mask);
	}

	mutex_unlock(&dev->mutex);
	return mask;
}

static ssize_t platform_button_write(struct file *filp, const char __user *buf, size_t count, loff_t * ppos)
{
	int val;
	//struct platform_button_dev *dev = container_of(filp->private_data, struct platform_button_dev, miscdev);
	struct platform_button_dev *dev = filp->private_data;
	printk(KERN_INFO "dev address:%d\n", dev);
	printk(KERN_INFO "pld address:%d\n", pld);
	
//	int ret;
	 
//	DECLARE_WAITQUEUE(wait, current);
/*  
	mutex_lock(&dev->mutex);
	add_wait_queue(&dev->w_wait, &wait);

	//printk("first_drv_write\n");

	while (dev->flag == 0) {
		if (filp->f_flags & O_NONBLOCK){
			ret = -EAGAIN;
			goto out;
		}

		__set_current_state(TASK_INTERRUPTIBLE);
		
		mutex_unlock(&dev->mutex);

		schedule();

		if (signal_pending(current)){
			ret = -ERESTARTSYS;
			goto out2;
		}

		mutex_lock(&dev->mutex);
	}
*/
	printk(KERN_INFO "copy_from_user begin\n");
	copy_from_user(&val, buf, count); //	copy_to_user();
	printk(KERN_INFO "copy_from_user end\n");

	if (val == 1)
	{
		// 点灯
		//*gpfdat &= ~((1<<4) | (1<<5) | (1<<6));
		s3c2410_gpio_setpin(S3C2410_GPF4, 0);
		s3c2410_gpio_setpin(S3C2410_GPF5, 0);
		s3c2410_gpio_setpin(S3C2410_GPF6, 0);
		dev->flag = 1;
		wake_up_interruptible(&dev->r_wait);
	}
	else
	{
		// 灭灯
		//*gpfdat |= (1<<4) | (1<<5) | (1<<6);
		s3c2410_gpio_setpin(S3C2410_GPF4, 1);
		s3c2410_gpio_setpin(S3C2410_GPF5, 1);
		s3c2410_gpio_setpin(S3C2410_GPF6, 1);
		dev->flag = 0;
	}
	
	//wake_up_interruptible(&dev->r_wait);
/*  
	if (dev->async_queue){
		kill_fasync(&dev->async_queue, SIGIO, POLL_IN);
		printk(KERN_DEBUG "%s kill SIGIO\n", __func__);
	}
*/
out:
	//mutex_unlock(&dev->mutex);
//out2:
//	remove_wait_queue(&dev->w_wait, &wait);
//	set_current_state(TASK_RUNNING);
	return 0;
}

static ssize_t platform_button_read(struct file *filp, char __user *buf, size_t count, loff_t * ppos)
{
	//int val;
	//struct platform_button_dev *dev = container_of(filp->private_data, struct platform_button_dev, miscdev);
	struct platform_button_dev *dev = filp->private_data;

	//int ret;
	
	/*  
	DECLARE_WAITQUEUE(wait, current);

	mutex_lock(&dev->mutex);
	add_wait_queue(&dev->r_wait, &wait);

	//printk("first_drv_write\n");
  
	while (dev->flag == 0) {
	// while (*gpfdat & ((1<<4) | (1<<5) | (1<<6))) {
		if (filp->f_flags & O_NONBLOCK){
			ret = -EAGAIN;
			goto out;
		}

		__set_current_state(TASK_INTERRUPTIBLE);
		
		mutex_unlock(&dev->mutex);

		schedule();

		if (signal_pending(current)){
			ret = -ERESTARTSYS;
			goto out2;
		}

		mutex_lock(&dev->mutex);
	}
	*/
	/*
	if (*gpfdat & ((1<<4) | (1<<5) | (1<<6)))
	{
		val = 0;
		printk(KERN_INFO "\nval = %d\n", val);
	}
	else
	{
		val = 1;
		printk(KERN_INFO "\nval = %d\n", val);
	}
	*/

	if (count != 1)
		return -EINVAL;

	wait_event_interruptible(button_waitq, ev_press);

	printk(KERN_INFO "copy_to_user begin\n");
	copy_to_user(buf, &key_state, count); //	copy_to_user();
	printk(KERN_INFO "copy_to_user end\n");
	//wake_up_interruptible(&dev->r_wait);
	ev_press = 0;
/*  
	if (dev->async_queue){
		kill_fasync(&dev->async_queue, SIGIO, POLL_IN);
		printk(KERN_DEBUG "%s kill SIGIO\n", __func__);
	}
*/
//out:
//	mutex_unlock(&dev->mutex);
//out2:
//	remove_wait_queue(&dev->r_wait, &wait);
//	set_current_state(TASK_RUNNING);
//	return 0;
	return 1;
}

int platform_button_release(struct inode *inode, struct file *filp)
{
	free_irq(IRQ_EINT11, &pins_desc[0]);
	free_irq(IRQ_EINT19, &pins_desc[1]);
	
	destroy_workqueue(myworkqueue);

	return 0;
}

static struct file_operations platform_button_fops = {
	.owner 		= THIS_MODULE,
	.open  		= platform_button_open,
	.read  		= platform_button_read,
	.write 		= platform_button_write,
	.poll  		= platform_button_poll,
	.release 	= platform_button_release,
	
};
/* 
static struct file_operations first_drv_fops = {
    .owner  =   THIS_MODULE,    // 这是一个宏，推向编译模块时自动创建的__this_module变量 
    .open   =   first_drv_open,     
	.write	=	first_drv_write,	   
};
*/


static int platform_button_probe(struct platform_device *pdev)
{
	//struct platform_button_dev *pl;
	int ret;

	pld = devm_kzalloc(&pdev->dev, sizeof(*pld), GFP_KERNEL);
	if(!pld)
		return -ENOMEM;
	pld->miscdev.minor = MISC_DYNAMIC_MINOR;
	pld->miscdev.name = "platform_button";
	pld->miscdev.fops = &platform_button_fops;

	mutex_init(&pld->mutex);
	init_waitqueue_head(&pld->r_wait);
//	init_waitqueue_head(&pld->w_wait);
	platform_set_drvdata(pdev, pld);

	ret = misc_register(&pld->miscdev);
	if (ret < 0)
		goto err;

	//gpfcon = (volatile unsigned long *)ioremap(0x56000050, 16);
	//gpfdat = gpfcon + 1;
	
	//gpgcon = (volatile unsigned long *)ioremap(0x56000060, 16);
	//gpgdat = gpgcon + 1;

	dev_info(&pdev->dev, "platform_button drv probed\n");

	return 0;
err:
	return ret;
}

static int platform_button_remove(struct platform_device *pdev)
{
	struct platform_button_dev *pl = platform_get_drvdata(pdev);

	misc_deregister(&pl->miscdev);

	dev_info(&pdev->dev, "platform_button dev removed\n");

	//iounmap(gpfcon);
	//iounmap(gpgcon);

	return 0;
}
/*  
int major;
static int first_drv_init(void)
{
	major = register_chrdev(0, "first_drv", &first_drv_fops); // 注册, 告诉内核

	firstdrv_class = class_create(THIS_MODULE, "firstdrv");

	firstdrv_class_dev = class_device_create(firstdrv_class, NULL, MKDEV(major, 0), NULL, "xyz"); // /dev/xyz 

	gpfcon = (volatile unsigned long *)ioremap(0x56000050, 16);
	gpfdat = gpfcon + 1;

	return 0;
}
static void first_drv_exit(void)
{
	unregister_chrdev(major, "first_drv"); // 卸载

	class_device_unregister(firstdrv_class_dev);
	class_destroy(firstdrv_class);
	iounmap(gpfcon);
}

*/
static struct platform_driver platform_button_driver = {
	.driver = {
		.name = "platform_button",
		.bus = &platform_bus_type,
		.owner = THIS_MODULE,
	},
	.probe = platform_button_probe,
	.remove = platform_button_remove,
};

static int __init platform_button_init(void)
{
	return platform_driver_register(&platform_button_driver);
}
module_init(platform_button_init);

static int __exit platform_button_exit(void)
{
	platform_driver_unregister(&platform_button_driver);
}
module_exit(platform_button_exit);

// module_platform_driver(platform_button_driver);


//module_init(first_drv_init);
//module_exit(first_drv_exit);

MODULE_AUTHOR("WL");
MODULE_LICENSE("GPL");
