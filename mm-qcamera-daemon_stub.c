#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>

int main() {
  umask(0000);
  execl("/system/bin/mm-qcamera-daemon_real","mm-qcamera-daemon_real",(char *)NULL);
  return 0;
}
