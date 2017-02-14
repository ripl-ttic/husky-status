#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <unistd.h>
#include <inttypes.h>
#include <string.h>
#define _GNU_SOURCE
#include <getopt.h>
#include <glib.h>

#include <bot_core/bot_core.h>
#include <bot_param/param_client.h>

#include <lcmtypes/bot_core.h>
#include <lcmtypes/hr_lcmtypes.h>
#include <lcmtypes/ros2lcm_husky_status_t.h>

#if DEBUG
#define dbg(...) do { fprintf(stderr, "[%s:%d] ", __FILE__, __LINE__); \
                      fprintf(stderr, __VA_ARGS__); } while(0)
#else
#define dbg(...) 
#endif


typedef struct _state_t {
    lcm_t *lcm;
    BotParam *param;
    char *name;
    int verbose;
    ros2lcm_husky_status_t *status_prev;
} state_t;



static void on_husky_status(const lcm_recv_buf_t *rbuf, const char *channel,
                            const ros2lcm_husky_status_t *msg, void *user)
{
    state_t *self = (state_t*) user;
    if(self->status_prev == NULL)
        self->status_prev = ros2lcm_husky_status_t_copy(msg);

    int64_t faults = 0;
    erlcm_robot_state_command_t state_cmd_msg;
    state_cmd_msg.utime = bot_timestamp_now();
    state_cmd_msg.sender = "husky-status";
    state_cmd_msg.comment = "";
    state_cmd_msg.fault_mask = ERLCM_ROBOT_STATUS_T_FAULT_MASK_NO_CHANGE;
    state_cmd_msg.state = ERLCM_ROBOT_STATE_COMMAND_T_STATE_STOP;

    if (msg->e_stop) {
        if (self->verbose)
            fprintf (stdout, "E-stop enabled --- Setting fault\n");
        faults |= ERLCM_ROBOT_STATUS_T_FAULT_ESTOP;
    }
    if (msg->lockout) {
        if (self->verbose)
            fprintf (stdout, "Lockout enabled --- Setting fault\n");
        faults |= ERLCM_ROBOT_STATUS_T_FAULT_LOCKOUT;
    }
    if (msg->no_battery) {
        if (self->verbose)
            fprintf (stdout, "No battery indicated --- Setting fault\n");
        faults |= ERLCM_ROBOT_STATUS_T_FAULT_BATTERY;
    }

    state_cmd_msg.faults = faults;
    erlcm_robot_state_command_t_publish (self->lcm, "ROBOT_STATE_COMMAND", &state_cmd_msg);
    
    
}



static void
husky_status_destroy(state_t *self)
{
    if (self) 
        free(self);
}

state_t *
husky_status_create()
{
    state_t *self = (state_t*)calloc(1, sizeof(state_t));
    if (!self) {
        dbg("Error: husky_status_create() failed to allocate self\n");
        goto fail;
    }
    self->name = strdup("HUSKY_STATUS");

    /* LCM */
    self->lcm = bot_lcm_get_global (NULL);
    if (!self->lcm) {
        dbg("Error: husky_status_create() failed to get global lcm object\n");
        goto fail;
    }
    
    bot_glib_mainloop_attach_lcm (self->lcm);

    // Subscribe to LCM messages
    ros2lcm_husky_status_t_subscribe (self->lcm, "HUSKY_STATUS", on_husky_status, self);

    //g_timeout_add(1000.0/PUBLISH_STATE_HZ, on_timer, self);

    return self; 
fail:
    husky_status_destroy(self);
    return NULL;
}


static void usage(int argc, char ** argv)
{
    fprintf (stderr, "Usage: %s [OPTIONS]\n"
             "Process to monitor Husky status\n"
             "\n"
             "Options:\n"
             "  -h, --help             shows this help text and exits\n"
             "  -v, --verbose          be verbose\n", argv[0]);
}


int main(int argc, char ** argv)
{
    setlinebuf (stdout);
    
    char *optstring = "hv";
    char c;
    struct option long_opts[] = {
        {"help", no_argument, 0, 'h'},
        {"verbose", no_argument, 0, 'v'},
        {0, 0, 0, 0}};

    state_t *self = husky_status_create();
    if (!self)
        return 1;
    
    while ((c = getopt_long (argc, argv, optstring, long_opts, 0)) >= 0)
    {
        switch (c) 
        {
        case 'v':
            self->verbose = 1;
            break;
        case 'h':
        default:
            usage(argc, argv);
            return 1;
        }
    }


    /* Main Loop */
    GMainLoop *main_loop = g_main_loop_new(NULL, FALSE);
    if (!main_loop) {
        dbg("Error: Failed to create the main loop\n");
        return -1;
    }

    /* sit and wait for messages */
    g_main_loop_run(main_loop);
    
    /* clean */
    husky_status_destroy(self);
    return 0;
}
