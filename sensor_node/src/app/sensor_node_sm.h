#ifndef SENSOR_NODE_SM_H
#define SENSOR_NODE_SM_H


/* List of end node states */
enum endnode_state { NotSynced, Synced };


// State machine initialization prototype
void sensor_node_sm_init(void);

void sensor_node_sm_set_state(enum endnode_state state);

// State machine processing prototype
int sensor_node_sm_run(void);

#endif