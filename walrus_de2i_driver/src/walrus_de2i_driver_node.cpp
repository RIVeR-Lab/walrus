// Based on http://rijndael.ece.vt.edu/de2i150/

#include <time.h>
#include <sys/time.h>
#include <unistd.h>
#include <stdint.h>
#include <fcntl.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/mman.h>
#include <ros/ros.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Bool.h>

// PCI CONSTANTS
#define PCI_VENDOR_ID 0x1172
#define PCI_DEVICE_ID 0x0004
#define MMAP_SIZE 65536

// ADDRESS MAP (read from QSYS)
#define PCI_MEMORY 0XC000
#define PCI_HEXPORT 0XC000
#define PCI_INPORT 0XC020

// finds PCI base address by vendor & device id
static off_t pci_read_base_address(int vendor, int device)
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

class PciMemory {
public:
  PciMemory() : fd(-1), pci_mem(NULL) {}

  bool open(int vendor, int device) {
    int fd = ::open("/dev/mem", O_RDWR|O_SYNC);
    off_t pci_bar0 = pci_read_base_address(vendor, device);
    pci_mem = (uint8_t*)mmap(0, MMAP_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, pci_bar0);

    if(pci_mem == MAP_FAILED) {
      perror("MMAP FAILED");
      pci_mem = NULL;
      close();
      return false;
    }
    else {
      return true;
    }
  }

  void close() {
    if(pci_mem) {
      munmap((void*)pci_mem, MMAP_SIZE);
      pci_mem = NULL;
    }
    if(fd != -1) {
      ::close(fd);
      fd = -1;
    }
  }

  uint16_t read16(int offset){
    return *(volatile uint16_t*)(pci_mem + offset);
  }

  void write32(int offset, uint32_t value){
    *(volatile uint32_t*)(pci_mem + offset) = value;
  }

private:
  int fd;
  volatile uint8_t *pci_mem;
};


void sprayCallback(PciMemory* fpga_mem, const std_msgs::Bool::ConstPtr data) {
  uint32_t raw = data->data?0x1:0x0;
  fpga_mem->write32(PCI_HEXPORT, raw);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "de2i_driver");

  PciMemory fpga_mem;
  if(!fpga_mem.open(PCI_VENDOR_ID, PCI_DEVICE_ID)) {
    ROS_FATAL("Failed to open FPGA PCI device");
  }

  ros::NodeHandle nh;

  ros::Publisher input_pub = nh.advertise<std_msgs::UInt16>("switches", 1);
  ros::Subscriber spray_sub = nh.subscribe<std_msgs::Bool>("spray", 1, boost::bind(sprayCallback, &fpga_mem, _1));

  ros::Rate rate(10);
  while(ros::ok()) {
    std_msgs::UInt16 msg;
    msg.data = fpga_mem.read16(PCI_INPORT);
    input_pub.publish(msg);
    ros::spinOnce();
    rate.sleep();
  }

  fpga_mem.close();

  return 0;
}

