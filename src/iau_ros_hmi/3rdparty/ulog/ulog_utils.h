#ifndef ULOG_UTILS_H
#define ULOG_UTILS_H

#include "ulog.h"
#include <stdlib.h>
#include <time.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

static int _ulog_default_log(void *userdata, int tag, const char *line)
{
    int rc = printf("%s", line);
    if (userdata) {
        FILE *fp = userdata;
        fputs(line, fp);
        fflush(fp);
    }
    return rc;
}

#define ENV_ULOG_LEVEL  "ULOG_LEVEL"
#define ENV_ULOG_DIR    "ULOG_DIR"
#ifdef _WIN32
# define CHAR_PATH_SP '\\'
#else
# define CHAR_PATH_SP '/'
#endif
// init ulog with default configuration.
// return if init OK. 1 => ok, 0 => error.
// will log to stdout and a log file in current work directory(If ENV var ULOG_DIR set to a directory, then use it).
// app: argv[0] program name, using for log file name. 'app.log.date_time'
// using filter if ENV var ULOG_LEVEL set Trace/Debug/Info/Warn/Error.
static int ulog_init_default(const char *app)
{
    int lv = ULOG_LV_ALL;
    char *env = getenv(ENV_ULOG_LEVEL);
    if (env) {
        if (strcasecmp(env, "Trace") == 0) {
            lv = ULOG_LV_TRACE;
        } else if (strcasecmp(env, "Debug") == 0) {
            lv = ULOG_LV_DEBUG;
        } else if (strcasecmp(env, "Info") == 0) {
            lv = ULOG_LV_INFO;
        } else if (strcasecmp(env, "Warn") == 0) {
            lv = ULOG_LV_WARN;
        } else if (strcasecmp(env, "Error") == 0) {
            lv = ULOG_LV_ERROR;
        } else {
            LOG_DEBUG("invalid ulog ENV '%s' = '%s', using default",
                     ENV_ULOG_LEVEL, env);
            env = "Trace";
        }
        LOG_INFO("ulog level=%s", env);
    }
    char wdir[256] = "";
    env = getenv(ENV_ULOG_DIR);
    if (env) {
        snprintf(wdir, sizeof(wdir) - 1, "%s", env);
        if (wdir[strlen(wdir) - 1] != CHAR_PATH_SP) {
            wdir[strlen(wdir) + 1] = 0;
            wdir[strlen(wdir)] = CHAR_PATH_SP;
        }
        LOG_DEBUG("ulog dir='%s'", wdir);
    }

    for (const char *p = app; *p; p++) {
        if (*p == '/' || *p == '\\') {
            app = p + 1;
        }
    }
    time_t now = time(NULL);
    char tim[32];
    strftime(tim, sizeof(tim), "%Y_%m%d_%H%M", localtime(&now));

    char buf[256];
    snprintf(buf, sizeof(buf), "%s%s.log.%s", wdir, app, tim);

    FILE* logf = fopen(buf, "w");
    if (logf) {
        LOG_INFO("ulog create log file '%s' OK", buf);
        ulog_init(_ulog_default_log, logf, lv);
        return 1;
    } else {
        LOG_INFO("ulog create log file '%s' FAILED, %s", buf, strerror(errno));
        ulog_init(NULL, NULL, lv);
        return 0;
    }
}

#ifdef __cplusplus
}
#endif
#endif // ULOG_UTILS_H
