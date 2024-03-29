#include <modules/pubsub/pubsub.h>
#include <modules/worker_thread/worker_thread.h>

WORKER_THREAD_TAKEOVER_MAIN(lpwork_thread, LOWPRIO)
WORKER_THREAD_SPAWN(can_thread, LOWPRIO+1, 1024)

PUBSUB_TOPIC_GROUP_CREATE(default_topic_group, 1024)
