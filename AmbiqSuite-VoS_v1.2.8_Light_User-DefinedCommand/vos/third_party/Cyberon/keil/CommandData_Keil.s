
    AREA _CMDData, DATA, READONLY

    EXPORT  u32CMDDataBegin
    EXPORT  u32CMDDataEnd

u32CMDDataBegin
	INCBIN .\UCLEAR_EngCmd_20210709_pack_WithTriAndMapID.bin.BITwave
u32CMDDataEnd

	END