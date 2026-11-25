#define _POSIX_C_SOURCE 200809L
#include <sys/ioctl.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
struct session {int32_t ret,handle;uint64_t data;};
#define OP _IOWR('T',1,struct session)
int main(int argc,char **argv){
 int f=open("/dev/gz_kree",O_RDWR|O_CLOEXEC);if(f<0){perror("open");return 1;}
 if(argc>1&&!strcmp(argv[1],"hold")){
  int r=open("/data/local/tmp/apcp-gz-main-ready",O_CREAT|O_TRUNC|O_WRONLY,0600);if(r<0)return 2;close(r);
  struct timespec t={.tv_sec=5};nanosleep(&t,0);close(f);puts("CLOSED");return 0;
 }
 errno=0;int r=ioctl(f,_IO('X',255),0);if(r!=-1||errno!=EINVAL)return 3;
 errno=0;r=ioctl(f,_IO('T',255),0);if(r!=-1||errno!=EINVAL)return 4;
 errno=0;r=ioctl(f,OP,NULL);if(r!=-1||errno!=EFAULT)return 5;
 close(f);
 for(int i=0;i<32;i++){f=open("/dev/gz_kree",O_RDWR|O_CLOEXEC);if(f<0)return 6;if(close(f))return 7;}
 puts("PASS: native endpoint open/close and unknown/null-request rejection; no firmware session or TA command");return 0;
}
