
    AREA _CMDData, DATA, READONLY

    EXPORT  u32CMDDataBegin
    EXPORT  u32CMDDataEnd

u32CMDDataBegin
	INCBIN .\UCLEAR_Voice_Commands_pack_WithMapID.bin
u32CMDDataEnd

	END