#include <fcntl.h>
#include <poll.h>
#include <unistd.h>
#include <stdio.h>
int main(void) {
 int f=open("/proc/gz_log",O_RDONLY|O_NONBLOCK);
 if(f<0){perror("open");return 1;}
 int ready=open("/data/local/tmp/apcp-gz-log-reader-ready",O_CREAT|O_WRONLY|O_TRUNC,0600);
 if(ready<0){close(f);return 2;}
 close(ready);
 struct pollfd p={.fd=f,.events=POLLIN};
 for(int i=0;i<20;i++) {poll(&p,1,20); usleep(200000);}
 close(f);puts("READER_CLOSED");return 0;
}
