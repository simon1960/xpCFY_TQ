/* Linux services used by the original control code. LONG stays 32 bits on LP64. */
#ifndef TQ_PLATFORM_H
#define TQ_PLATFORM_H
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <strings.h>
#include <errno.h>
#include <pthread.h>
#include <time.h>
#include <limits.h>
#include <unistd.h>
#include <sys/stat.h>
typedef int32_t LONG;
typedef unsigned long DWORD;
typedef uint32_t UINT;
typedef uint64_t ULONGLONG;
typedef int64_t LONGLONG;
typedef void* LPVOID;
#define WINAPI
#define TRUE 1
#define FALSE 0
#define MAX_PATH PATH_MAX
#define INFINITE ((DWORD)-1)
#define WAIT_OBJECT_0 0UL
#define WAIT_TIMEOUT 258UL
#define WAIT_FAILED ((DWORD)-1)
#define INVALID_FILE_ATTRIBUTES ((DWORD)-1)
#define FILE_ATTRIBUTE_DIRECTORY 16UL
#define ERROR_FILE_NOT_FOUND ENOENT
#define MOVEFILE_REPLACE_EXISTING 1
#define MOVEFILE_WRITE_THROUGH 2
#define _TRUNCATE ((size_t)-1)
#define _stricmp strcasecmp
#define MB_OK 0x00000000UL
#define MB_ICONWARNING 0x00000030UL
#define MB_ICONINFORMATION 0x00000040UL
#define MB_TASKMODAL 0x00002000UL
#define MB_SETFOREGROUND 0x00010000UL
typedef pthread_rwlock_t SRWLOCK;
#define SRWLOCK_INIT PTHREAD_RWLOCK_INITIALIZER
#define AcquireSRWLockExclusive pthread_rwlock_wrlock
#define ReleaseSRWLockExclusive pthread_rwlock_unlock
#define AcquireSRWLockShared pthread_rwlock_rdlock
#define ReleaseSRWLockShared pthread_rwlock_unlock
static inline LONG InterlockedExchange(volatile LONG* p, LONG v) { return (__atomic_exchange_n(p,v,__ATOMIC_SEQ_CST)); }
static inline LONG InterlockedCompareExchange(volatile LONG* p, LONG v, LONG expected) { __atomic_compare_exchange_n(p,&expected,v,0,__ATOMIC_SEQ_CST,__ATOMIC_SEQ_CST); return (expected); }
static inline LONG InterlockedIncrement(volatile LONG* p) { return (__atomic_add_fetch(p,1,__ATOMIC_SEQ_CST)); }
static inline LONG InterlockedOr(volatile LONG* p, LONG v) { return (__atomic_fetch_or(p,v,__ATOMIC_SEQ_CST)); }
static inline DWORD GetLastError(void) { return (DWORD)errno; }
static inline int fopen_s(FILE** f, const char* path, const char* mode) { *f=fopen(path,mode); return (*f?0:errno); }
static inline int strcpy_s(char* d, size_t n, const char* s) { if(!d||!s||!n) return (EINVAL); if(strlen(s)>=n) { d[0]=0; return (ERANGE); } memcpy(d,s,strlen(s)+1); return (0); }
static inline int strncpy_s(char* d, size_t n, const char* s, size_t count) { if(!d||!s||!n) return (EINVAL); size_t len=strlen(s); if(len>count) len=count; if(len>=n) len=n-1; memcpy(d,s,len); d[len]=0; return (0); }
static inline int DeleteFileA(const char* p) { return (unlink(p)==0); }
static inline DWORD GetFileAttributesA(const char* p) { struct stat s; return (stat(p,&s)?INVALID_FILE_ATTRIBUTES:(S_ISDIR(s.st_mode)?FILE_ATTRIBUTE_DIRECTORY:0)); }
int MoveFileExA(const char* from, const char* to, unsigned flags);
ULONGLONG GetTickCount64(void);
void Sleep(DWORD ms);
typedef struct { unsigned wDay,wMonth,wYear,wHour,wMinute,wSecond,wMilliseconds; } SYSTEMTIME;
void GetLocalTime(SYSTEMTIME* out);
static inline void OutputDebugStringA(const char* s) { fputs(s,stderr); }
UINT GetPrivateProfileIntA(const char* section,const char* key,UINT fallback,const char* path);
DWORD GetPrivateProfileStringA(const char* section,const char* key,const char* fallback,char* out,DWORD size,const char* path);
int MessageBoxA(void* parent,const char* text,const char* title,unsigned int type);
typedef struct tq_handle* HANDLE;
HANDLE CreateEventA(void* security,int manual,int initial,const char* name);
HANDLE CreateThread(void* security,size_t stack,DWORD (*fn)(void*),void* argument,DWORD flags,void* id);
int SetEvent(HANDLE h);
DWORD WaitForSingleObject(HANDLE h,DWORD ms);
int CloseHandle(HANDLE h);
#endif
