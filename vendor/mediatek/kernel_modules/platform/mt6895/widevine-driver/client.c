#define _POSIX_C_SOURCE 200809L
#include <sys/ioctl.h>
#include <fcntl.h>
#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
struct param {int share_fd;uint64_t sec_handle;};
struct param32 {int share_fd;uint32_t sec_handle;};
#define CMD32 _IOR('W',1,struct param32)
#define CMD _IOR('W',1,struct param)
int main(int argc,char **argv){
 int fd=open("/dev/drm_wv",O_RDWR|O_CLOEXEC);if(fd<0){perror("open");return 1;}
 if(argc>1&&!strcmp(argv[1],"hold")){
  int ready=open("/data/local/tmp/apcp-widevine-ready",O_CREAT|O_TRUNC|O_WRONLY,0600);if(ready<0)return 2;close(ready);struct timespec t={.tv_sec=5};nanosleep(&t,0);
  errno=0;int r=ioctl(fd,_IO('X',255),0);if(r!=-1||errno!=EINVAL)return 3;close(fd);puts("HELD_FD_SAFE_AFTER_UNBIND");return 0;
 }
 struct param p={.share_fd=-1};errno=0;int r=ioctl(fd,CMD,&p);if(r!=-1||errno!=EFAULT)return 4;
 errno=0;r=ioctl(fd,CMD,NULL);if(r!=-1||errno!=EFAULT)return 5;
 errno=0;r=ioctl(fd,_IO('X',255),0);if(r!=-1||errno!=EINVAL)return 6;
 int plain=open("/dev/null",O_RDONLY|O_CLOEXEC);if(plain<0)return 7;p.share_fd=plain;errno=0;r=ioctl(fd,CMD,&p);close(plain);if(r!=-1||errno!=EFAULT)return 8;
 struct param32 n={.share_fd=-1};errno=0;r=ioctl(fd,CMD32,&n);if(r!=-1||errno!=EFAULT)return 9;
 errno=0;r=ioctl(fd,CMD32,NULL);if(r!=-1||errno!=EFAULT)return 10;
 close(fd);puts("PASS_BOTH_ABI_INVALID_FD_NULL_UNKNOWN_AND_NON_DMABUF; no protected buffer or keys");return 0;
}
