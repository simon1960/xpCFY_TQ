#include "platform.h"
#include <stdlib.h>
#include <ctype.h>
#include <fcntl.h>
#include <sys/wait.h>

ULONGLONG GetTickCount64(void) { struct timespec t; clock_gettime(CLOCK_MONOTONIC,&t); return (uint64_t)t.tv_sec*1000+t.tv_nsec/1000000; }
void Sleep(DWORD ms) { struct timespec t={(time_t)(ms/1000),(long)(ms%1000)*1000000}; while(nanosleep(&t,&t)&&errno==EINTR) {} }
void GetLocalTime(SYSTEMTIME* out) { struct timespec t; struct tm tm; clock_gettime(CLOCK_REALTIME,&t); localtime_r(&t.tv_sec,&tm); *out=(SYSTEMTIME){tm.tm_mday,tm.tm_mon+1,tm.tm_year+1900,tm.tm_hour,tm.tm_min,tm.tm_sec,(unsigned)(t.tv_nsec/1000000)}; }
int MoveFileExA(const char* from,const char* to,unsigned flags) {
    (void)flags;
    int fd=open(from,O_RDONLY); if(fd<0) return (0);
    int rc=fsync(fd), saved=errno; close(fd); if(rc) { errno=saved; return (0); }
    if(rename(from,to)) return (0);
    char parent[PATH_MAX]; if(strcpy_s(parent,sizeof(parent),to)) { errno=ENAMETOOLONG; return (0); }
    char* slash=strrchr(parent,'/'); if(slash) { if(slash==parent) slash[1]=0; else *slash=0; } else strcpy(parent,".");
    fd=open(parent,O_RDONLY|O_DIRECTORY); if(fd<0) return (0);
    rc=fsync(fd); saved=errno; close(fd); if(rc) errno=saved; return (rc==0);
}
static char* trim(char* s) { while(isspace((unsigned char)*s)) ++s; char* e=s+strlen(s); while(e>s&&isspace((unsigned char)e[-1])) --e; *e=0; return (s); }
DWORD GetPrivateProfileStringA(const char* section,const char* key,const char* fallback,char* out,DWORD size,const char* path) {
    if(!size) return (0);
    strncpy_s(out,size,fallback,_TRUNCATE);
    FILE* f=fopen(path,"r"); if(!f) return (strlen(out));
    char* line=NULL; size_t capacity=0; int active=0;
    while(getline(&line,&capacity,f)>=0) {
        char* s=trim(line); if(*s=='#'||*s==';'||!*s) continue;
        if(*s=='[') { char* end=strchr(s+1,']'); active=0; if(end) { *end=0; active=!strcasecmp(trim(s+1),section); } continue; }
        if(!active) continue;
        char* equal=strchr(s,'='); if(!equal) continue; *equal=0;
        if(strcasecmp(trim(s),key)) continue;
        char* value=trim(equal+1); size_t n=strlen(value);
        if(n>=2 && (value[0]=='\"'||value[0]=='\'') && value[n-1]==value[0]) { value[n-1]=0; ++value; }
        strncpy_s(out,size,value,_TRUNCATE); break;
    }
    free(line); fclose(f); return (strlen(out));
}
UINT GetPrivateProfileIntA(const char* section,const char* key,UINT fallback,const char* path) {
    char text[64],*end; GetPrivateProfileStringA(section,key,"",text,sizeof(text),path);
    if(!*text||*text=='-') return (fallback);
    errno=0; unsigned long long v=strtoull(text,&end,10);
    return (errno||*end||v>UINT32_MAX?fallback:(UINT)v);
}
int MessageBoxA(void* parent,const char* text,const char* title,unsigned int type) {
    pid_t child;
    int status=0;
    const char* mode=(type&MB_ICONWARNING)?"--warning":"--info";
    (void)parent;
    child=fork();
    if(child<0) { fprintf(stderr,"%s: %s\n",title,text); return (0); }
    if(child==0) { execlp("zenity","zenity",mode,"--title",title,"--text",text,(char*)NULL); fprintf(stderr,"%s: %s\n",title,text); _exit(127); }
    while(waitpid(child,&status,0)<0&&errno==EINTR) {}
    return (WIFEXITED(status)?WEXITSTATUS(status):0);
}
/* Manual-reset stop events and joinable workers use monotonic condition waits.
   No pthread cancellation: the hardware worker must run its motor-safe cleanup. */
struct tq_handle { pthread_mutex_t mutex; pthread_cond_t condition; pthread_t thread; int is_thread,signalled,joined; DWORD (*fn)(void*); void* argument; };
static HANDLE make_handle(void) {
    HANDLE h=calloc(1,sizeof(*h)); if(!h) return (NULL);
    int rc=pthread_mutex_init(&h->mutex,NULL); if(rc) { free(h); errno=rc; return (NULL); }
    pthread_condattr_t attr; rc=pthread_condattr_init(&attr);
    if(rc) { pthread_mutex_destroy(&h->mutex); free(h); errno=rc; return (NULL); }
    rc=pthread_condattr_setclock(&attr,CLOCK_MONOTONIC);
    if(!rc) rc=pthread_cond_init(&h->condition,&attr);
    pthread_condattr_destroy(&attr);
    if(rc) { pthread_mutex_destroy(&h->mutex); free(h); errno=rc; return (NULL); }
    return (h);
}
HANDLE CreateEventA(void* security,int manual,int initial,const char* name) { (void)security; (void)name; if(!manual) { errno=EINVAL; return (NULL); } HANDLE h=make_handle(); if(h) h->signalled=initial; return (h); }
int SetEvent(HANDLE h) { if(!h) return (0); pthread_mutex_lock(&h->mutex); h->signalled=1; pthread_cond_broadcast(&h->condition); pthread_mutex_unlock(&h->mutex); return (1); }
static void* thread_entry(void* arg) { HANDLE h=arg; h->fn(h->argument); SetEvent(h); return (NULL); }
HANDLE CreateThread(void* security,size_t stack,DWORD (*fn)(void*),void* argument,DWORD flags,void* id) {
    (void)security; (void)stack; (void)flags; (void)id;
    HANDLE h=make_handle(); if(!h) return (NULL); h->is_thread=1; h->fn=fn; h->argument=argument;
    int rc=pthread_create(&h->thread,NULL,thread_entry,h);
    if(rc) { h->is_thread=0; CloseHandle(h); errno=rc; return (NULL); } return (h);
}
DWORD WaitForSingleObject(HANDLE h,DWORD ms) {
    if(!h) return (WAIT_FAILED);
    struct timespec deadline; clock_gettime(CLOCK_MONOTONIC,&deadline);
    if(ms!=INFINITE) { deadline.tv_sec+=ms/1000; deadline.tv_nsec+=(ms%1000)*1000000; if(deadline.tv_nsec>=1000000000) { ++deadline.tv_sec; deadline.tv_nsec-=1000000000; } }
    pthread_mutex_lock(&h->mutex); int rc=0;
    while(!h->signalled && !rc) rc=ms==INFINITE?pthread_cond_wait(&h->condition,&h->mutex):pthread_cond_timedwait(&h->condition,&h->mutex,&deadline);
    int ready=h->signalled; pthread_mutex_unlock(&h->mutex);
    if(ready && h->is_thread && !h->joined) { rc=pthread_join(h->thread,NULL); if(rc) { errno=rc; return (WAIT_FAILED); } h->joined=1; }
    if(ready) return (WAIT_OBJECT_0);
    if(rc==ETIMEDOUT) return (WAIT_TIMEOUT);
    errno=rc; return (WAIT_FAILED);
}
int CloseHandle(HANDLE h) { if(!h) return (0); if(h->is_thread&&!h->joined) { errno=EBUSY; return (0); } pthread_cond_destroy(&h->condition); pthread_mutex_destroy(&h->mutex); free(h); return (1); }
