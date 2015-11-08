#define MEMORY_OFFSET (0x6000)

typedef enum {
	COMMADDRESS_READY = MEMORY_OFFSET,
	COMMADDRESS_HALF_READY = COMMADDRESS_READY +1,
	COMMADDRESS_OBJECTS_COUNT = COMMADDRESS_HALF_READY + 1,
	COMMADDRESS_CORE_KEY = COMMADDRESS_OBJECTS_COUNT + 1,
	COMMADDRESS_OBJECTS = COMMADDRESS_CORE_KEY + 1,
	COMMADDRESS_FRAMES = COMMADDRESS_OBJECTS+300*4, //this value is hard-coded

} CommAddress;

typedef enum {
	MEMORYBANK_BANK1 = 0x2000,
	MEMORYBANK_BANK2 = 0x4000,
	MEMORYBANK_BANK3 = 0x6000
} MemoryBanks;

typedef enum {

	CORE_0_0_MEM = 0x80800000,
	CORE_0_1_MEM = 0x80900000,
	CORE_0_2_MEM = 0x80a00000,
	CORE_0_3_MEM = 0x80b00000,
	CORE_1_0_MEM = 0x84800000,
	CORE_1_1_MEM = 0x84900000,
	CORE_1_2_MEM = 0x84a00000,
	CORE_1_3_MEM = 0x84b00000,
	CORE_2_0_MEM = 0x88800000,
	CORE_2_1_MEM = 0x88900000,
	CORE_2_2_MEM = 0x88a00000,
	CORE_2_3_MEM = 0x88b00000,
	CORE_3_0_MEM = 0x8c800000,
	CORE_3_1_MEM = 0x8c900000,
	CORE_3_2_MEM = 0x8ca00000,
	CORE_3_3_MEM = 0x8cb00000,
} CoreAddress;
