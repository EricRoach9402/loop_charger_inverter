/**
 * @file inverter_alarm_manager.c
 * @brief Alarm Manager implementation – currently logs only.
 *
 * This is the single place to extend when alarm behaviour grows beyond
 * logging (CMOS publish, MQTT, DB write, escalation policy, rate limiting).
 * alarm_engine.c and inverter_alarm.c never need to change as this grows.
 */

#include <errno.h>
#include <sqlite3.h>
#include <string.h>
#include <sys/stat.h>
#include <time.h>

#include "inverter_alarm_manager.h"
#include "log.h"

#define LOG_DIR       "/run/media/mmcblk1p1/Logs/inverter_alarm"
#define LOG_PATH      "/run/media/mmcblk1p1/Logs/inverter_alarm/alarms.db"
#define LOG_NODE      "inverter_alarm_node"
#define LOG_MAX_ROWS  10000

static sqlite3 *g_db = NULL;
static int      g_db_open_failed = 0;

/* ensure parent directory exists before sqlite3_open */
static void ensure_log_dir(void)
{
    if (mkdir(LOG_DIR, 0755) != 0 && errno != EEXIST) {
        LOG_ERROR("[Alarm DB] cannot create directory '%s': %s",
                  LOG_DIR, strerror(errno));
    }
}

/* initialize SQLite logging – idempotent, safe to call multiple times */
static void log_init(void)
{
    if (g_db) {
        return;
    }

    if (g_db_open_failed) {
        return;
    }

    ensure_log_dir();

    if (sqlite3_open(LOG_PATH, &g_db) != SQLITE_OK) {
        LOG_ERROR("[Alarm DB] failed to open '%s': %s",
                  LOG_PATH, sqlite3_errmsg(g_db));
        sqlite3_close(g_db);
        g_db = NULL;
        g_db_open_failed = 1;
        return;
    }

    char *errmsg = NULL;
    if (sqlite3_exec(g_db,
            "CREATE TABLE IF NOT EXISTS logs ("
            "  id        INTEGER PRIMARY KEY,"
            "  timestamp TEXT NOT NULL,"
            "  node      TEXT NOT NULL,"
            "  event     TEXT NOT NULL,"
            "  detail    TEXT NOT NULL);",
            NULL, NULL, &errmsg) != SQLITE_OK) {
        LOG_ERROR("[Alarm DB] CREATE TABLE failed: %s",
                  errmsg ? errmsg : "unknown");
        sqlite3_free(errmsg);
        sqlite3_close(g_db);
        g_db = NULL;
        g_db_open_failed = 1;
        return;
    }

    LOG_INFO("[Alarm DB] opened '%s'.", LOG_PATH);
}

/* write log entry to SQLite */
static void write_log(const char *event, const char *detail)
{
    if (!g_db) return;

    time_t    now = time(NULL);
    struct tm tm_buf;
    char      ts[32];
    strftime(ts, sizeof(ts), "%Y-%m-%d %H:%M:%S", localtime_r(&now, &tm_buf));

    int count = 0;
    sqlite3_stmt *cnt = NULL;
    if (sqlite3_prepare_v2(g_db, "SELECT COUNT(*) FROM logs;", -1, &cnt, NULL) == SQLITE_OK) {
        if (sqlite3_step(cnt) == SQLITE_ROW)
            count = sqlite3_column_int(cnt, 0);
        sqlite3_finalize(cnt);
    }

    const char *sql;
    if (count < LOG_MAX_ROWS) {
        sql = "INSERT INTO logs (timestamp, node, event, detail) VALUES (?, ?, ?, ?);";
    } else {
        sql = "UPDATE logs SET timestamp=?, node=?, event=?, detail=?"
              " WHERE id=(SELECT id FROM logs ORDER BY timestamp ASC, id ASC LIMIT 1);";
    }

    sqlite3_stmt *stmt = NULL;
    if (sqlite3_prepare_v2(g_db, sql, -1, &stmt, NULL) != SQLITE_OK) return;

    sqlite3_bind_text(stmt, 1, ts,       -1, SQLITE_TRANSIENT);
    sqlite3_bind_text(stmt, 2, LOG_NODE, -1, SQLITE_STATIC);
    sqlite3_bind_text(stmt, 3, event,    -1, SQLITE_STATIC);
    sqlite3_bind_text(stmt, 4, detail,   -1, SQLITE_STATIC);

    if (sqlite3_step(stmt) != SQLITE_DONE) {
        LOG_ERROR("[Alarm DB] write failed: %s", sqlite3_errmsg(g_db));
    }

    sqlite3_finalize(stmt);
}

/* ── Public API ───────────────────────────────────────────────────────── */

void inverter_alarm_manager_close(void)
{
    if (g_db) {
        sqlite3_close(g_db);
        g_db = NULL;
    }
}

void inverter_alarm_manager_handle_event(const alarm_entry_t *entry,
                                         uint16_t              value,
                                         alarm_event_t         event,
                                         void                 *userdata)
{
    const module_config_t *cfg = (const module_config_t *)userdata;
    const char *unit_name = cfg ? cfg->name : "unknown";

    log_init();
    char detail[128];

    switch (event) {
    case ALARM_EVENT_TRIGGER:
        LOG_WARNING("[Alarm] %s | err=0x%04X | value=0x%04X | %s",
                    unit_name, entry->error_code, value, entry->description);

        snprintf(detail, sizeof(detail), "err=0x%04X | value=0x%04X | %s",
                 entry->error_code, value, entry->description);

        write_log("ALARM_TRIGGER", detail);
        break;

    case ALARM_EVENT_CLEAR:
        LOG_INFO("[Alarm] %s | CLEARED err=0x%04X | value=0x%04X | %s",
                 unit_name, entry->error_code, value, entry->description);

        snprintf(detail, sizeof(detail), "CLEARED err=0x%04X | value=0x%04X | %s",
                 entry->error_code, value, entry->description);

        write_log("ALARM_CLEAR", detail);
        break;
    }
}
