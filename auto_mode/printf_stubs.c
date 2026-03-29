#include <errno.h>
#include <sys/stat.h>
#include <sys/times.h>
#include <sys/types.h>
#include <sys/unistd.h>
#include <stdint.h>

#include "debug_uart.h"

#undef errno
extern int errno;

char *__env[1] = { 0 };
char **environ = __env;

extern unsigned int _StackTop;
extern unsigned int _HeapStart;

#define STACKSIZE 0x1000

void _exit(int status)
{
    (void)status;

    while (1)
    {
    }
}

int _close(int file)
{
    (void)file;
    return -1;
}

int _execve(char *name, char **argv, char **env)
{
    (void)name;
    (void)argv;
    (void)env;
    errno = ENOMEM;
    return -1;
}

int _fork(void)
{
    errno = EAGAIN;
    return -1;
}

int _fstat(int file, struct stat *st)
{
    (void)file;
    st->st_mode = S_IFCHR;
    return 0;
}

int _getpid(void)
{
    return 1;
}

int _kill(int pid, int sig)
{
    (void)pid;
    (void)sig;
    errno = EINVAL;
    return -1;
}

int _link(char *old, char *new)
{
    (void)old;
    (void)new;
    errno = EMLINK;
    return -1;
}

int _lseek(int file, int ptr, int dir)
{
    (void)file;
    (void)ptr;
    (void)dir;
    return 0;
}

int _stat(const char *filepath, struct stat *st)
{
    (void)filepath;
    st->st_mode = S_IFCHR;
    return 0;
}

clock_t _times(struct tms *buf)
{
    (void)buf;
    return (clock_t)-1;
}

int _unlink(char *name)
{
    (void)name;
    errno = ENOENT;
    return -1;
}

int _wait(int *status)
{
    (void)status;
    errno = ECHILD;
    return -1;
}

int _isatty(int file)
{
    switch (file)
    {
        case STDOUT_FILENO:
        case STDERR_FILENO:
        case STDIN_FILENO:
            return 1;

        default:
            errno = EBADF;
            return 0;
    }
}

caddr_t _sbrk(int incr)
{
    char *heap_start;
    char *stack_end;
    static char *heap_end = 0;
    char *previous_heap_end;

    heap_start = (char *)&_HeapStart;
    stack_end = (char *)((uintptr_t)&_StackTop - STACKSIZE);

    if (heap_end == 0)
    {
        heap_end = heap_start;
    }

    previous_heap_end = heap_end;

    if ((heap_end + incr) > stack_end)
    {
        errno = ENOMEM;
        return (caddr_t)-1;
    }

    heap_end += incr;
    return (caddr_t)previous_heap_end;
}

int _read(int file, char *ptr, int len)
{
    (void)file;
    (void)ptr;
    (void)len;
    return 0;
}

int _write(int file, char *ptr, int len)
{
    int index;

    switch (file)
    {
        case STDOUT_FILENO:
        case STDERR_FILENO:
            for (index = 0; index < len; ++index)
            {
                debug_uart_write_char(ptr[index]);
            }
            return len;

        default:
            errno = EBADF;
            return -1;
    }
}
