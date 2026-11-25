#define _GNU_SOURCE
#include <dirent.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <ctype.h>
#include <errno.h>
int main(int argc,char **argv){
 if(argc!=2)return 2;DIR *p=opendir("/proc");if(!p)return 2;struct dirent *e;int busy=0;
 while((e=readdir(p))){if(!isdigit(e->d_name[0]))continue;char d[128];snprintf(d,sizeof(d),"/proc/%s/fd",e->d_name);DIR *f=opendir(d);if(!f){if(errno==EACCES){closedir(p);return 2;}continue;}struct dirent *q;
 while((q=readdir(f))){if(q->d_name[0]=='.')continue;char path[512],target[512];snprintf(path,sizeof(path),"%s/%s",d,q->d_name);ssize_t n=readlink(path,target,sizeof(target)-1);if(n<0)continue;target[n]=0;if(!strcmp(target,argv[1])){puts(path);busy=1;}}
 closedir(f);}closedir(p);puts(busy?"BUSY":"NO_OPENERS");return busy;
}
