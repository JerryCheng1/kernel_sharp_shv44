#include <linux/export.h>
#include <linux/module.h>
#include <linux/notifier.h>

static BLOCKING_NOTIFIER_HEAD(touchevent_notifier_list_touchevent);

int touchevent_register_client(struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&touchevent_notifier_list_touchevent, nb);
}
EXPORT_SYMBOL(touchevent_register_client);

int touchevent_unregister_client(struct notifier_block *nb)
{
	return blocking_notifier_chain_unregister(&touchevent_notifier_list_touchevent, nb);
}
EXPORT_SYMBOL(touchevent_unregister_client);

int touchevent_notifier_call_chain(unsigned long val, void *v)
{
	return blocking_notifier_call_chain(&touchevent_notifier_list_touchevent, val, v);
}
EXPORT_SYMBOL(touchevent_notifier_call_chain);

static int __init touchevent_notifier_init(void)
{
	return 0;
}

static void __exit touchevent_notifier_exit(void)
{
	return;
}

module_init(touchevent_notifier_init);
module_exit(touchevent_notifier_exit);

MODULE_VERSION("1.0");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("SHARP CORPORATION");
MODULE_DESCRIPTION("TOUCH EVENT NOTIFIER");
