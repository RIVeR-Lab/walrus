// Based on http://rijndael.ece.vt.edu/de2i150/

#include <time.h>
#include <sys/time.h>
#include <unistd.h>
#include <stdint.h>
#include <fcntl.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/mman.h>

// PCI CONSTANTS
#define PCI_VENDOR_ID 0x1172
#define PCI_DEVICE_ID 0x0004
#define MMAP_SIZE 65536

// ADDRESS MAP (read from QSYS)
#define PCI_OUTPORT 0XC040
#define PCI_INPORT 0XC060
#define PCI_MEMORY 0XC000

// finds PCI base address by vendor & device id
off_t pci_read_base_address(int vendor, int device)
{
	FILE* f;
	int dev, dum;
	unsigned int mem;
	char buf[0x1000];

	f = fopen("/proc/bus/pci/devices", "r");
	if (!f) return 0;

	while (fgets(buf, sizeof(buf)-1, f)) {
		if (sscanf(buf,"%x %x %x %x", &dum, &dev, &dum, &mem) != 4) continue;
		if (dev == ((vendor<<16) | device)) {
			fclose(f);
			return (off_t)mem;
		}
	}
	fclose(f);
	return 0;
}

inline void pci_mm_read(uint8_t* dst, uint8_t* devptr, int offset, int n){
  int i;
  for(i=0; i<n; i++)
    *(uint8_t*)(dst+i) = *(uint8_t*)(devptr+offset+i);
}

inline void pci_mm_write(uint8_t* src, uint8_t* devptr, int offset, int n){
  int i;
  for(i=0; i<n; i++)
    *(uint8_t*)(devptr+offset+i) = *(uint8_t*)(src+i);
}

int main(){
  int fd = open("/dev/mem", O_RDWR|O_SYNC);
  off_t pci_bar0 = pci_read_base_address(PCI_VENDOR_ID, PCI_DEVICE_ID);
  uint8_t* ptr = (uint8_t*)mmap(0, MMAP_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, pci_bar0);

  if(ptr == MAP_FAILED)
    perror("MMAP FAILED");
  else
    printf("PCI BAR0 0x0000 = %p",  ptr);

  munmap(ptr, MMAP_SIZE);
  close(fd);

  return 0;
}

