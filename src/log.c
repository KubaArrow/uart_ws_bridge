/*
 * Copyright (c) 2020 rxi
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 */

#include "uart_ws_bridge/log.h"

#ifdef _WIN32
#include <windows.h>
static LARGE_INTEGER log_frequency;
void log_init_timer() {
    QueryPerformanceFrequency(&log_frequency);
}
#else
#include <sys/time.h>
#endif

#define MAX_CALLBACKS 32

typedef struct
{
  log_LogFn fn;
  void *udata;
  int level;
} Callback;

static struct
{
  void *udata;
  log_LockFn lock;
  int level;
  bool quiet;
  Callback callbacks[MAX_CALLBACKS];
} L;

static const char *level_strings[] = {
    "TRACE", "DEBUG", "INFO", "WARN", "ERROR", "FATAL"};

#ifdef LOG_USE_COLOR
static const char *level_colors[] = {
    "\x1b[94m", "\x1b[36m", "\x1b[32m", "\x1b[33m", "\x1b[31m", "\x1b[35m"};
#endif

void log_get_timestamp(char *buffer, size_t size) {
#ifdef USE_HIGHRES_TIME
    #ifdef _WIN32
        static LARGE_INTEGER start_counter;
        static time_t start_time = 0;
        static int initialized = 0;

        if (!initialized) {
            QueryPerformanceFrequency(&log_frequency);
            QueryPerformanceCounter(&start_counter);
            start_time = time(NULL);
            initialized = 1;
        }

        LARGE_INTEGER current_counter;
        QueryPerformanceCounter(&current_counter);

        LONGLONG elapsed = current_counter.QuadPart - start_counter.QuadPart;
        double delta_seconds = (double)elapsed / (double)log_frequency.QuadPart;
        time_t current_time = start_time + (time_t)delta_seconds;

        struct tm tm_time;
        localtime_s(&tm_time, &current_time);

        int usec = (int)((delta_seconds - (int)delta_seconds) * 1000000.0);

        snprintf(buffer, size, "[%04d-%02d-%02d %02d:%02d:%02d.%04d]",
                 tm_time.tm_year + 1900, tm_time.tm_mon + 1, tm_time.tm_mday,
                 tm_time.tm_hour, tm_time.tm_min, tm_time.tm_sec, usec/100);
    #else
        struct timeval tv;
        gettimeofday(&tv, NULL);
        struct tm tm_time;
        localtime_r(&tv.tv_sec, &tm_time);

        snprintf(buffer, size, "[%04d-%02d-%02d %02d:%02d:%02d.%04ld]",
                 tm_time.tm_year + 1900, tm_time.tm_mon + 1, tm_time.tm_mday,
                 tm_time.tm_hour, tm_time.tm_min, tm_time.tm_sec,
                 (long)tv.tv_usec/100);
    #endif
#else
    time_t t = time(NULL);
    struct tm tm_time;
    #ifdef _WIN32
    localtime_s(&tm_time, &t);
    #else
    localtime_r(&t, &tm_time);
    #endif
    strftime(buffer, size, "[%Y-%m-%d %H:%M:%S]", &tm_time);
#endif
}

static void stdout_callback(log_Event *ev)
{
  char timestamp[64];
  log_get_timestamp(timestamp, sizeof(timestamp));

#ifdef LOG_USE_COLOR
  fprintf(
      ev->udata, "%s %s%-5s\x1b[0m \x1b[90m%s:%d:\x1b[0m ",
      timestamp, level_colors[ev->level], level_strings[ev->level],
      ev->file, ev->line);
#else
  fprintf(
      ev->udata, "%s %-5s %s:%d: ",
      timestamp, level_strings[ev->level], ev->file, ev->line);
#endif

  vfprintf(ev->udata, ev->fmt, ev->ap);
  fflush(ev->udata);
}

static void file_callback(log_Event *ev)
{
  char timestamp[64];
  log_get_timestamp(timestamp, sizeof(timestamp));

  fprintf(ev->udata, "%s %-5s %s:%d: ",
          timestamp, level_strings[ev->level], ev->file, ev->line);
  vfprintf(ev->udata, ev->fmt, ev->ap);
  fflush(ev->udata);
}

static void lock(void)
{
  if (L.lock)
  {
    L.lock(true, L.udata);
  }
}

static void unlock(void)
{
  if (L.lock)
  {
    L.lock(false, L.udata);
  }
}

const char *log_level_string(int level)
{
  return level_strings[level];
}

void log_set_lock(log_LockFn fn, void *udata)
{
  L.lock = fn;
  L.udata = udata;
}

void log_set_level(int level)
{
  L.level = level;
}

void log_set_quiet(bool enable)
{
  L.quiet = enable;
}

int log_add_callback(log_LogFn fn, void *udata, int level)
{
  for (int i = 0; i < MAX_CALLBACKS; i++)
  {
    if (!L.callbacks[i].fn)
    {
      L.callbacks[i] = (Callback){fn, udata, level};
      return 0;
    }
  }
  return -1;
}

int log_add_fp(FILE *fp, int level)
{
  return log_add_callback(file_callback, fp, level);
}

static void init_event(log_Event *ev, void *udata)
{
  if (!ev->time)
  {
    time_t t = time(NULL);
    ev->time = localtime(&t);
  }
  ev->udata = udata;
}

void log_log(int level, const char *file, int line, const char *fmt, ...)
{
  log_Event ev = {
      .fmt = fmt,
      .file = file,
      .line = line,
      .level = level,
  };

  lock();

  if (!L.quiet && level >= L.level)
  {
    init_event(&ev, stderr);
    va_start(ev.ap, fmt);
    stdout_callback(&ev);
    va_end(ev.ap);
  }

  for (int i = 0; i < MAX_CALLBACKS && L.callbacks[i].fn; i++)
  {
    Callback *cb = &L.callbacks[i];
    if (level >= cb->level)
    {
      init_event(&ev, cb->udata);
      va_start(ev.ap, fmt);
      cb->fn(&ev);
      va_end(ev.ap);
    }
  }

  unlock();
}
