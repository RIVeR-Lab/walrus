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
#define PCI_MEMORY 0XC000
#define PCI_HEXPORT 0XC000
#define PCI_INPORT 0XC020

unsigned char hexdigit[] = {0x3F, 0x06, 0x5B, 0x4F,
                            0x66, 0x6D, 0x7D, 0x07,
                            0x7F, 0x6F, 0x77, 0x7C,
			    0x39, 0x5E, 0x79, 0x71};

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

inline void pci_mm_read16(uint8_t* dst, uint8_t* devptr, int offset, int n){
  for(int i=0; i<n; i+=2) {
    *(uint16_t*)(dst+i) = *(uint16_t*)(devptr+offset+i);
  }
}

inline void pci_mm_write32(uint8_t* src, uint8_t* devptr, int offset, int n){
  for(int i=0; i<n; i +=4) {
    *(uint32_t*)(devptr+offset+i) = *(uint32_t*)(src+i);
  }
}

int main(){
  int fd = open("/dev/mem", O_RDWR|O_SYNC);
  off_t pci_bar0 = pci_read_base_address(PCI_VENDOR_ID, PCI_DEVICE_ID);
  uint8_t* ptr = (uint8_t*)mmap(0, MMAP_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, pci_bar0);

  if(ptr == MAP_FAILED)
    perror("MMAP FAILED");
  else
    printf("PCI BAR0 0x0000 = %p\n",  ptr);


  while(true) {
    uint16_t j;
    pci_mm_read16((uint8_t*)&j, ptr, PCI_INPORT, 2);

    uint32_t k = hexdigit[j & 0xF]
      | (hexdigit[(j >>  4) & 0xF] << 8)
      | (hexdigit[(j >>  8) & 0xF] << 16)
      | (hexdigit[(j >> 12) & 0xF] << 24);
    k = ~k;
    pci_mm_write32((uint8_t*)&k, ptr, PCI_HEXPORT, 4);
  }

  munmap(ptr, MMAP_SIZE);
  close(fd);

  return 0;
}

