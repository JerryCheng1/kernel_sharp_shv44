#ifndef _TOUCHEVENT_NOTIFIER_H_
#define _TOUCHEVENT_NOTIFIER_H_

#include <linux/notifier.h>

int touchevent_register_client(struct notifier_block *nb);
int touchevent_unregister_client(struct notifier_block *nb);

int touchevent_notifier_call_chain(unsigned long val, void *v);

#endif /* _TOUCHEVENT_NOTIFIER_H_ */
