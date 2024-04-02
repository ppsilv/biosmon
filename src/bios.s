;***********************************************************************
; SERIAL 16c550 DRIVER
; AUTHOR: Paulo da Silva (pgordao)
;
; Version: 0.0.2

.setcpu "6502"

.segment "ZEROPAGE"
RACC     = $30               ;;: .res 1
RPHY     = $31               ;;: .res 1
RPHX     = $32               ;;: .res 1
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; These variable is just for test of UART.
MSGL     = $33
MSGH     = $34
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; These variables is to save registers X and Y in cpu 6502 although it does
; not have PHX and PHY.
MPHX     = $35
MPHY     = $36
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; Counter is to be used com DEC in 6502 that does not has a dec for Acc
COUNTER  = $37
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
LAST_CMD = $38  
ADDR1L   = $39          ; Digito 4 A do hexa 0xABCD
ADDR1H   = $3A          ; Digito 3 B do hexa 0xABCD
ADDR2L   = $3B          ; Digito 2 C do hexa 0xABCD
ADDR2H   = $3C          ; Digito 1 D do hexa 0xABCD
BSZ      = $3D          ; string size in buffer 
ERRO     = $3E          ; CODIGO DO ERRO
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;Flag to sign the use of WRITE_BYTE or WRITE_BYTE_LF
MEOR     = $3F
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
TMP      = $40          ;;TEMPORARY REGISTERS
TMP1     = $41
TMP2     = $42

BIN      = $0200          ; Buffer size = 128 bytes

;0 - No errors
;1 - Erro na conversao hexadecimal

.segment "DRV"

;Uart registers
PORT = $7800            ;;Uart address
R_RX = $00    ;;receiver buffer register (read only)
R_TX = $00    ;;transmitter holding register (write only)
RDLL = $00    ;;divisor latch LSB (if DLAB=1)
RDLH = $01    ;;divisor latch HSB (if DLAB=1)
RIER = $01    ;;interrupt enable register
RIIR = $02    ;;interrupt identification register
RFCR = $02    ;;FIFO control register
RLCR = $03    ;;line control register
RMCR = $04    ;;modem control register
RLSR = $05    ;;line status register
RMSR = $06    ;;modem status register
RSCR = $07	;;scratch register

; Constants
.if .not .def(CR)
	CR  = $0D ; Carriage Return
.endif	
.if .not .def(LF)
	LF  = $0A ; Line feed
.endif	

DIV_4800_LO   = 24
DIV_4800_HI   = 0
DIV_9600_LO   = 12
DIV_9600_HI   = 0
DIV_19200_LO  = 6
DIV_19200_HI  = 0
DIV_115200_LO = 1
DIV_115200_HI = 0
POLLED_MODE   = %00000000
LCR_8N1       = %00000011
DLAB          = %10000000
FIFO_ENABLE   = %00000111 ;%00000111
THR_EMPTY     = %01100000       ;;

DATA_READY  = %00000001
OVERRUN_ERR = %00000010
PARITY_ERR  = %00000100
FRAMING_ERR = %00001000
BREAK_INT   = %00010000
MCR_DTR  = $01  ;dtr output 
MCR_RTS  = $02  ;rts output 
MCR_OUT1 = $04  ;output #1  
MCR_OUT2 = $08  ;output #2  
MCR_LOOP = $10  ;loop back  
MCR_AFCE = $20  ;auto flow control enable

;RESET:
;    JMP     RESET1

INITUART:
    LDA        #DLAB               ;set the divisor latch access bit (DLAB)
    STA        PORT+RLCR
    LDA        #DIV_9600_LO        ;store divisor low byte (9600 baud @ 1,8 MHz clock)
    STA        PORT+RDLL
    LDA        #DIV_9600_HI        ;store divisor hi byte                               
    STA        PORT+RDLH
    LDA        #FIFO_ENABLE        ;enable the UART FIFO
    STA        PORT+RFCR
    LDA        #POLLED_MODE	       ;disable all interrupts
    STA        PORT+RIER
	LDA        #LCR_8N1            ;set 8 data bits, 1 stop bit, no parity, disable DLAB
    STA        PORT+RLCR
    LDA        #MCR_OUT2 + MCR_RTS + MCR_DTR + MCR_AFCE
    STA        PORT+RMCR 
    LDA        PORT+R_RX           ;Clear RX buffer      
    RTS	

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; B_READ_BYTE: Read byte from UART waiting for it (BLOCANT)
; Registers changed: A
; Flag CARRY not changed.
;
B_READ_BYTE:
	LDA PORT+RLSR 												;// check the line status register:
	AND #(OVERRUN_ERR | PARITY_ERR | FRAMING_ERR | BREAK_INT)   ; check for errors
	BEQ @NO_ERR 												    ;// if no error bits, are set, no error
	LDA PORT+R_RX
	JMP B_READ_BYTE
@NO_ERR:
	LDA PORT+RLSR 												    ;// reload the line status register
	AND #DATA_READY
	BEQ B_READ_BYTE   											;// if data ready is not set, loop
	LDA PORT+R_RX
	RTS

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; B_READ_BYTE_ECHO: Read byte from UART waiting for it (BLOCANT) with echo
; Registers changed: A
; Flag CARRY not changed.
;
B_READ_BYTE_ECHO:
    JSR B_READ_BYTE
;ECHO CHAR
    JSR WRITE_BYTE
;*********
	RTS

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; READ_BYTE: Read byte from UART waiting for it (NO BLOCANT)
; Registers changed: A, Y
; Flag CARRY: Set when character ready
;             Clear when no character ready
READ_BYTE:
    STY     MPHY                ; Save Y Reg
	LDA PORT+RLSR 												;// check the line status register:
	AND #(OVERRUN_ERR | PARITY_ERR | FRAMING_ERR | BREAK_INT)   ; check for errors
	BEQ @NO_ERR 												    ;// if no error bits, are set, no error
	LDA PORT+R_RX
	JMP NO_CHAR ;READ_BYTE
@NO_ERR:
	LDA PORT+RLSR 												    ;// reload the line status register
	AND #DATA_READY
	BEQ NO_CHAR   											;// if data ready is not set, loop
	LDA PORT+R_RX
    LDY     #$FF
@txdelay:
    DEY
    BNE     @txdelay
    LDY     MPHY
	SEC		    										;// otherwise, we have data! Load it. 				    									;// clear the carry flag to indicate no error
	RTS
NO_CHAR:
    LDY     #$FF
@txdelay1:
    DEY
    BNE     @txdelay1
    LDY     MPHY
    CLC
    RTS

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; READ_BYTE_ECHO: Read byte from UART waiting for it (NO BLOCANT) with echo
; Registers changed: A, Y
; Flag CARRY: Set when character ready
;             Clear when no character ready
READ_BYTE_ECHO:
    JSR     READ_BYTE
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;ECHO CHAR
    JSR     WRITE_BYTE
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
    RTS


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; WRITE_BYTE: Write byte to UART
; Registers changed: NONE
; Flag CARRY not changed.
;
WRITE_BYTE:
    STY     MPHY                ; Save Y Reg
    STX     MPHX                ; Save X Reg
    PHA                         ; Save A Reg
WAIT_FOR_THR_EMPTY:
    LDA     PORT+RLSR           ; Get the Line Status Register
    AND     #THR_EMPTY          ; Check for TX empty
    BEQ     WAIT_FOR_THR_EMPTY 	; loop while the THR is not empty
	PLA
	STA     PORT+R_TX 			; send the byte
    PHA
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;DELAY BETWEEN CHAR SENT
    LDY     #$10
@YDELAY:
    LDA     #$FF
    STA     COUNTER
@txdelay:
    DEC     COUNTER
    BNE     @txdelay
    DEY
    BNE     @YDELAY
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
    PLA
    LDX     MPHX                ; Restore X Reg
    LDY     MPHY                ; Restore Y Reg
    RTS

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; WRITE_BYTE_WITH_LF: Write byte to UART, IF BYTE IS 0D WRITE 0A(LF) TOO
; Registers changed: NONE
; Flag CARRY not changed.
;
WRITE_BYTE_WITH_LF:
    PHA
    JSR     WRITE_BYTE
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;WRITE A LF IF ACC HAS A $0D IN IT
    PLA
    CMP     #$0D
    BNE     WRITE_BYTE_WITH_ECHO_FIM
    LDA     #$0A
    JSR     WRITE_BYTE
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
WRITE_BYTE_WITH_ECHO_FIM:
    RTS

.segment "BIOS"

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;                        RESET  
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
RESET:        
	            SEI					; No interrupt
	            CLD					; Set decimal
	            LDX #$FE 			; Set stack pointer
	            TXS

                ;;Initializing some variables
                LDA     #$00
                STA     ADDR1L
                STA     ADDR1H
                STA     ADDR2L
                STA     ADDR2H

                JSR     INITUART
                LDA     #<MSG1
                STA     MSGL
                LDA     #>MSG1
                STA     MSGH
                JSR     SHWMSG
NEXT_CHAR:
                LDA     #$0D
                JSR     WRITE_BYTE
                LDA     #'>'
                JSR     WRITE_BYTE
                
                JSR     B_READ_BYTE
                JSR     WRITE_BYTE

                ;CMP     #'S'            ;Show memory address data format: ADDR
                ;BEQ     TEMP_S
                CMP     #'D'            ;Dump de memoria format: ADDR:ADDR
                BEQ     TEMP_D
                CMP     #'M'            ;Put byte into memory address
                BEQ     TEMP_M
                CMP     #'R'            ;Run programa na format: ADDR R
                BEQ     TEMP_R
                CMP     #'H'            ;Show help 
                BEQ     TEMP_H
                JMP     NEXT_CHAR

TEMP_S:         JMP     DIGITOU_S
TEMP_D:         JMP     DIGITOU_D
TEMP_M:         JMP     DIGITOU_M
TEMP_R:         JMP     DIGITOU_R
TEMP_H:         JMP     DIGITOU_H
              
DIGITOU_S:      
                STA     LAST_CMD
                LDA     #<MSG2
                STA     MSGL
                LDA     #>MSG2
                STA     MSGH
                JSR     SHWMSG
                JSR     GETLINE
                LDA     #<BIN
                STA     MSGL
                LDA     #>BIN
                STA     MSGH
                JSR     SHWMSG
                RTS
                JMP     NEXT_CHAR
DIGITOU_D:
                STA     LAST_CMD
                LDA     #<MSG3
                STA     MSGL
                LDA     #>MSG3
                STA     MSGH
                JSR     SHWMSG
                JSR     GETLINE
                ;Get addr from 
                LDY     #$00    
                JSR     CONV_ADDR_TO_HEX
                LDX     TMP1
                LDY     TMP2
                JSR     SWAP_XY
                STX     ADDR1L
                STY     ADDR1H

                LDY     #$04   
                LDA     BIN,Y
                CMP     #$3E
                BNE     DIGITOU_D_SHOWMEM

                ;Get addr to 
                LDY     #$05    
                JSR     CONV_ADDR_TO_HEX
                LDX     TMP1
                LDY     TMP2
                JSR     SWAP_XY
                STX     ADDR2L
                STY     ADDR2H
                ;JSR     PRINT_ADDR_HEXA
                LDA     #$08
                STA     TMP2    
LINHA:          LDX     #$08
                LDA     #$0D
                JSR     WRITE_BYTE
                LDA     ADDR1H
                JSR     PRBYTE    
                LDA     ADDR1L
                JSR     PRBYTE    
                LDA     #' '
                JSR     WRITE_BYTE
DIGITOU_D_WORK:
                ;addressing mode of 65C02
                ;LDA     (ADDR1L)
                ;addressing mode of 6502
                LDY     #$0
                LDA     (ADDR1L),Y
                ;******************
                JSR     PRBYTE    
                LDA     #' '
                JSR     WRITE_BYTE
                JSR     INC_ADDR
                JSR     COMP_ADDR
                BEQ     DIGITOU_D_WORK
                BCS     DIGITOU_D_FIM
                ;JSR     PRINT_ADDR_HEXA                
                ;JSR     READ_BYTE
                DEX
                BEQ     LINHA
                JMP     DIGITOU_D_WORK
DIGITOU_D_FIM:
                JMP     NEXT_CHAR
DIGITOU_D_SHOWMEM:
                LDY     #$04   
                LDA     BIN,Y
                CMP     #$3A
                BEQ     DIGITOU_D_SHOWMEM_FIM
                LDA     ADDR1H
                JSR     PRBYTE    
                LDA     ADDR1L
                JSR     PRBYTE    
                LDA     #' '
                JSR     WRITE_BYTE
                ;addressing mode of 65C02
                ;LDA     (ADDR1L)
                ;addressing mode of 6502
                LDY     #$0
                LDA     (ADDR1L),Y

                JSR     PRBYTE    
DIGITOU_D_SHOWMEM_FIM:
                JMP     NEXT_CHAR

DIGITOU_M:      
                STA     LAST_CMD
                LDA     #<MSG7
                STA     MSGL
                LDA     #>MSG7
                STA     MSGH
                JSR     SHWMSG
                JSR     GETLINE
                ;Get addr from 
                LDY     #$00    
                JSR     CONV_ADDR_TO_HEX
                LDX     TMP1
                LDY     TMP2
                JSR     SWAP_XY
                STX     ADDR1L
                STY     ADDR1H

                ;VERIFICAR SE O COMANDO É :
                LDY     #$04   
                LDA     BIN,Y
                CMP     #$3A
                BNE     DIGITOU_M_FIM

                LDY     #$05
                LDA     BIN,Y
                JSR     ROL_LEFT    
                STA     TMP1
                INY
                LDA     BIN,Y
                JSR     NO_ROL_RIGHT
                ORA     TMP1
                STA     TMP1
                ;addressing mode of 65C02
                ;STA     (ADDR1L)
                ;addressing mode of 6502
                LDY     #$0
                STA     (ADDR1L),Y                

                ;LDA     ADDR1H
                ;JSR     PRBYTE
                ;LDA     ADDR1L
                ;JSR     PRBYTE
                ;LDA     #':'
                ;JSR     WRITE_BYTE
                ;LDA     TMP1
                ;JSR     PRBYTE
DIGITOU_M_FIM:                
                JMP     NEXT_CHAR

DIGITOU_H:
                STA     LAST_CMD
                LDA     #<HELP
                STA     MSGL
                LDA     #>HELP
                STA     MSGH
                JSR     SHWMSG
                JMP     NEXT_CHAR

DIGITOU_R:
                STA     LAST_CMD
                LDA     #<MSG4
                STA     MSGL
                LDA     #>MSG4
                STA     MSGH
                JSR     SHWMSG
                JSR     GETLINE

                ;;DEBUG
                JSR     PRINT_BUFFER


                LDY     #$00              
                JSR     CONV_ADDR_TO_HEX
                LDX     TMP1
                LDY     TMP2
                JSR     SWAP_XY
                STX     ADDR1L
                STY     ADDR1H
                ;;DEBUG
                JSR     PRINT_BUFFER_HEXA
                JMP     (ADDR1L)
                JMP     NEXT_CHAR
SWAP_XY:
                STY     TMP     ; Y 2 M
                TXA             ; X 2 A
                TAY             ; A 2 Y    
                LDX     TMP     ; M 2 X
                RTS
ROL_LEFT:
                JSR     CONV_HEX_1DIG
                BCC     CONV_HEX_4DIG_FIM
                ROL
                ROL
                ROL
                ROL
                AND     #$F0
                RTS
NO_ROL_RIGHT:
                JSR     CONV_HEX_1DIG
                BCC     CONV_HEX_4DIG_FIM
                AND     #$0F
                RTS
CONV_HEX_4DIG_FIM:
                LDA     #<MSG6
                STA     MSGL
                LDA     #>MSG6
                STA     MSGH
                JSR     SHWMSG
                LDA     #$01
                STA     ERRO
                CLC
                RTS                
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;CONV_ADDR_TO_HEX:
;
CONV_ADDR_TO_HEX:
                ;;Dig 4
                LDA     #'4'
                JSR     WRITE_BYTE
                LDA     BIN,Y
                JSR     WRITE_BYTE
                LDA     BIN,Y
                JSR     ROL_LEFT    
                STA     TMP1
                ;;Dig 3
                LDA     #'3'
                JSR     WRITE_BYTE
                INY
                LDA     BIN,Y
                JSR     WRITE_BYTE
                LDA     BIN,Y
                JSR     NO_ROL_RIGHT
                ORA     TMP1
                STA     TMP1
                ;;Dig 2
                LDA     #'2'
                JSR     WRITE_BYTE
                INY
                LDA     BIN,Y
                JSR     WRITE_BYTE
                LDA     BIN,Y
                JSR     ROL_LEFT
                STA     TMP
                ;;Dig 1
                LDA     #'1'
                JSR     WRITE_BYTE
                INY
                LDA     BIN,Y
                JSR     WRITE_BYTE
                LDA     BIN,Y
                JSR     NO_ROL_RIGHT
                ORA     TMP
                STA     TMP2

                SEC
                RTS

;*******************************************
;CONV_HEX_1DIG:  
;Parameter: A digit to be converted
;Return...: A digit converted
CONV_HEX_1DIG:  
                ;;DEBUG
                JSR     WRITE_BYTE
                CMP     #$30
                BCC     CONV_HEX_1DIG_FIM
                CMP     #$3A
                BCC     DIG_0_A_9
                CMP     #$41
                BCS     DIG_A_TO_Z
                ;CARACTER PODE SER UM DESSES : ; < = > ? @
                CLC     ;CLEAR CARRY FLAG DIG NOT CONVERTED
                RTS
DIG_A_TO_Z:     
                CMP     #$47
                BCS     CONV_HEX_1DIG_FIM
                CLC
                SBC     #$36
                SEC     ;SET CARRY FLAG DIG CONVERTED
                RTS                
DIG_0_A_9:
                CLC
                SBC     #$2F
                SEC     ;SET CARRY FLAG DIG CONVERTED
                RTS
CONV_HEX_1DIG_FIM:  
                CLC
                RTS
;********************************************
;Print 4 digits hexadecimal
PRINT_BUFFER_HEXA:
                LDA     #'['
                JSR     WRITE_BYTE
                LDA     ADDR1L
                JSR     PRBYTE
                LDA     ADDR1H
                JSR     PRBYTE
                LDA     LAST_CMD
                CMP     #'D'
                BNE     PRINT_ADDR_HEXA_FIM
                LDA     #'.'
                JSR     WRITE_BYTE
                LDA     ADDR2L
                JSR     PRBYTE
                LDA     ADDR2H
                JSR     PRBYTE
PRINT_ADDR_HEXA_FIM:                
                LDA     #']'
                JSR     WRITE_BYTE
                RTS
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
GETLINE:        LDX     #$00
GETLINE1:       JSR     READ_BYTE
                JSR     WRITE_BYTE
                STA     BIN,X
                INX
                CMP     #$0D
                BNE     GETLINE1     
                LDA     #$00
                STA     BIN,X
                STX     BSZ

                RTS
;*************************************************
;Print buffer
PRINT_BUFFER:
                LDA     #<BIN
                STA     MSGL
                LDA     #>BIN
                STA     MSGH
                JSR     PRBYTE
                LDA     #$0D
                JSR     WRITE_BYTE_WITH_LF
                RTS
;*************************************************
SHWMSG:         LDY #$0
SMSG:           LDA (MSGL),Y
                BEQ SMDONE
                JSR WRITE_BYTE_WITH_LF
                LDA #$FF
                STA COUNTER
@txdelay:
                DEC COUNTER
                BNE @txdelay
                INY
                BNE SMSG
SMDONE:         RTS

PRBYTE:     PHA             ;Save A for LSD.
            LSR
            LSR
            LSR             ;MSD to LSD position.
            LSR
            JSR PRHEX       ;Output hex digit.
            PLA             ;Restore A.
PRHEX:      AND #$0F        ;Mask LSD for hex print.
            ORA #$B0        ;Add "0".
            CMP #$BA        ;Digit?
            BCC ECHO        ;Yes, output it.
            ADC #$06        ;Add offset for letter.
ECHO:       PHA             ;*Save A
            AND #$7F        ;*Change to "standard ASCII"
            JSR  WRITE_BYTE
            PLA             ;*Restore A
            RTS             ;*Done, over and out...

;Incrementa endereco
INC_ADDR:
            CLC
            LDA #$01
            ADC ADDR1L
            STA ADDR1L
            LDA #$00
            ADC ADDR1H
            STA ADDR1H
            RTS
;Compara enderecos            
COMP_ADDR:
            LDA ADDR1H
            CMP ADDR2H
            BNE COMP_ADDR_FIM
            LDA ADDR1L
            CMP ADDR2L
COMP_ADDR_FIM:
            RTS


MSG1:            .byte CR,LF,"PDSILVA - BIOSMON 2024 - 0.1",CR,0
MSG2:            .byte CR,"Input Addr: ",CR,0
MSG3:            .byte CR,"Dump Mem. Addr: Fmt XXXX>XXXX or XXXX:",CR,0
MSG4:            .byte CR,"Run program in Addr: Format abcd",CR,0
MSG5:            .byte CR,"EXECUTADO",CR,0
MSG6:            .byte CR,"Hex conv. error",CR,0
MSG7:            .byte CR,"Poke: Fmt addr:dt",CR,0
HELP:            .byte CR,"Help BIOSMON v 0.1",CR,LF
                 .byte "Commands:",CR
                 .byte "         S - Put data into buffer",CR
                 .byte "         D - Dump memory",CR
                 .byte "         M - Poke",CR
                 .byte "         R - Run program",CR
                 .byte "         H - Show help",CR,LF,0

;Used just for test of run cmd. 

OLD_WOZ:
                LDA     #<MSG5
                STA     MSGL
                LDA     #>MSG5
                STA     MSGH
                JSR     SHWMSG
                JMP     NEXT_CHAR

.segment "RESETVEC"

                .word   RESET          ; NMI vector
                .word   RESET          ; RESET vector
                .word   $0000          ; IRQ vector
