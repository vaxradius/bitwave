
    AREA _CMDData, DATA, READONLY

    EXPORT  u32CMDDataBegin
    EXPORT  u32CMDDataEnd

u32CMDDataBegin
	INCBIN .\UCLEAR_EngCmd_20210706_pack_WithMapID.bin
u32CMDDataEnd

	END