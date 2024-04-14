;****************************************************************************
;----------------------------------------------------------------------------
; Monitor program source code for 6502 Microprocessor Kit
; Written by Paulo Da Silva(pgordao), ppsilv@gmail.com
; Copyright (c) 2024
; Serial 16c550 driver
;
; Version.: 0.0.5
;
; The version 0.0.3 does not has the change of read and write bytes to support msbasic.
; The version 0.0.5:
; Changed the absolute address to .res for variables in pagezero
; Added Fill command: F ADDR_FROM ADDR_TO VALUE
; Added Block command a block copy: B ADDR_FROM ADDR_TO   
; Changed the code structure all commands gets its own files .s
;

.setcpu "6502"


.segment "ZEROPAGE"
RACC    : .res 1  ;;= $30               ;;: .res 1
RPHY    : .res 1  ;;= $31               ;;: .res 1
RPHX    : .res 1  ;;= $32               ;;: .res 1
MSGL    : .res 1  ;;= $33
MSGH    : .res 1  ;;= $34
TMP     : .res 1  ;;= $35              ;;TEMPORARY REGISTERS
TMP1    : .res 1  ;;= $36
TMP2    : .res 1  ;;= $37
LAST_CMD: .res 1  ;;= $38  
ADDR1L  : .res 1  ;;= $39          ; Digito 4 A do hexa 0xABCD
ADDR1H  : .res 1  ;;= $3A          ; Digito 3 B do hexa 0xABCD
ADDR2L  : .res 1  ;;= $3B          ; Digito 2 C do hexa 0xABCD
ADDR2H  : .res 1  ;;= $3C          ; Digito 1 D do hexa 0xABCD
;LEN     : .res 2
BSZ     : .res 1  ;;= $3D          ; string size in buffer 
ERRO    : .res 1  ;;= $3E          ; CODIGO DO ERRO
COUNTER : .res 1  ;;= $3F
FLAGECHO: .res 1  ;;= $40          ; This flag must contain 00 to disable character echo


VERSION:    .byte "0.0.5"

.segment "BIOS"
BIN      = $200          ; Buffer size = 128 bytes
ERR01    = $01          ; Conversion error in CONV_HEX_4DIG_FIM   
ERR02    = $02          ; Syntax erro in cmd copy memory block.
ERR03    = $03          ; Syntax erro in fill memory block.

;*************************************************************
;*************************************************************
; RESET
;*************************************************************
;*************************************************************

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
                STA     FLAGECHO
                ;;Initialize PIA
                ;;JSR     INIT8255
                ;;Initialize ACIA
                JSR     INITUART
                LDA     #<MSG1
                STA     MSGL
                LDA     #>MSG1
                STA     MSGH
                JSR     SHWMSG
                ;;Turn off flag echo
                JSR     DIGITOU_AST
NEXT_CHAR:
;               LDX     #$FF
;                CPX     FLAGECHO
                ;BNE     NO_LF
                LDA     #$0D
                JSR     WRITE_BYTE
;NO_LF:
                LDA     #'>'
                JSR     WRITE_BYTE                
                JSR     READ_BYTE

                LDX     #$00
                CPX     FLAGECHO
                BNE     NO_ECHO
                JSR     WRITE_BYTE
NO_ECHO:
                CMP     #'*'            ;Turn on/off character echo
                BEQ     TEMP_AST
                CMP     #'B'            ;Copy memory block from source to dest
                BEQ     TEMP_B
                CMP     #'D'            ;Dump memory block from source to dest
                BEQ     TEMP_D
                CMP     #'F'            ;Fill memory block with a value
                BEQ     TEMP_F
                CMP     #'M'            ;(POKE)Put byte into memory address
                BEQ     TEMP_M
                CMP     #'P'            ;(PEEK)get byte frin memoria ADDR:ADDR
                BEQ     TEMP_P
                CMP     #'R'            ;Run programa na format: ADDR R
                BEQ     TEMP_R
                CMP     #'?'            ;Show help 
                BEQ     TEMP_H
                JMP     NEXT_CHAR
TEMP_B:         JMP     CMD_CP_BLOCK
TEMP_D:         JMP     CMD_DUMP
TEMP_F:         JMP     CMD_FILL
TEMP_M:         JMP     CMD_POKE
TEMP_P:         JMP     CMD_PEEK
TEMP_R:         JMP     CMD_RUN
TEMP_H:         JMP     CMD_HELP
TEMP_AST:       JMP     DIGITOU_AST     
      
              
.include "utils.s"
.include "cmd_cblock.s"
.include "cmd_dump.s"
.include "cmd_echo.s"
.include "cmd_fill.s"
.include "cmd_peek.s"
.include "cmd_poke.s"
.include "cmd_run.s"

SYNTAX_ERROR:
                LDA     #<MSG9
                STA     MSGL
                LDA     #>MSG9
                STA     MSGH
                JSR     SHWMSG
                JMP     NEXT_CHAR

CMD_HELP:
                STA     LAST_CMD
                LDA     #<HELP
                STA     MSGL
                LDA     #>HELP
                STA     MSGH
                JSR     SHWMSG
                JMP     NEXT_CHAR



;;ROL_LEFT_SAFE:       
;;                JSR     HEX_2_ASC_SAFE
;;                BCC     ROL_FIM
;;                ROL
;;                ROL
;;                ROL
;;                ROL
;;                AND     #$F0
;;                SEC
;;                RTS
;;GET_DIG_RIGHT_SAFE:
;;                JSR     HEX_2_ASC_SAFE
;;                BCC     ROL_FIM
;;                AND     #$0F
;;                SEC
;;                RTS
;;
;;ROL_FIM:
;;                LDA     #<MSG6
;;                STA     MSGL
;;                LDA     #>MSG6
;;                STA     MSGH
;;                JSR     SHWMSG
;;                LDA     ERR01
;;                STA     ERRO
;;                CLC
;;                RTS      
;;                    
;;
;;CONV_ADDR_TO_HEX_APAGARD_METODO_ANTIGO:
;;                ;;Dig 4
;;                LDA     BIN,Y
;;                JSR     ROL_LEFT    
;;                STA     TMP
;;                ;;Dig 3
;;                INY
;;                LDA     BIN,Y
;;                JSR     GET_DIG_RIGHT
;;                ORA     TMP
;;                STA     TMP1
;;                JSR     PRINT_PAR1
;;                JSR     PRBYTE
;;                ;;Dig 2
;;                INY
;;                LDA     BIN,Y
;;                JSR     ROL_LEFT
;;                STA     TMP
;;                ;;Dig 1
;;                INY
;;                LDA     BIN,Y
;;                JSR     GET_DIG_RIGHT
;;                ORA     TMP
;;                STA     TMP2
;;                JSR     PRBYTE
;;                JSR     PRINT_PAR2
;;
;;                SEC
;;                RTS
;;
;;;*******************************************
;HEX_2_ASC_SAFE:  
;Parameter: A digit to be converted
;Return...: A digit converted
;
;HEX_2_ASC_SAFE:
;;CONV_HEX_1DIG:  
;                CMP     #$30
;                BCC     CONV_HEX_1DIG_FIM
;                CMP     #$3A
;                BCC     DIG_0_A_9
;                CMP     #$41
;                BCS     DIG_A_TO_Z
;                ;CARACTER PODE SER UM DESSES : ; < = > ? @
;                CLC     ;CLEAR CARRY FLAG DIG NOT CONVERTED
;                RTS
;DIG_A_TO_Z:     
;                CMP     #$47
;                BCS     CONV_HEX_1DIG_FIM
;                SEC     ;Flag carry seted does not borrow     
;                SBC     #$37
;                SEC     ;SET CARRY FLAG TO SIGN DIG CONVERTED
;                RTS                
;DIG_0_A_9:
;                SEC      ;Flag carry seted does not borrow
;                SBC     #$30
;                SEC     ;SET CARRY FLAG TO SIGN DIG CONVERTED
;                RTS
;CONV_HEX_1DIG_FIM:  
;                CLC
;                RTS

MSG1:            .byte CR,LF,"PDSILVA - BIOSMON 2024 - Version: ",VERSION,CR,0
MSG2:            .byte "Input data: ",CR,0
MSG3:            .byte LF,"Dump Mem. Addr: Fmt XXXX.XXXX or XXXX:",CR,0
MSG4:            .byte "Run program in Addr: Format abcd",CR,0
MSG5:            .byte "EXECUTADO",CR,0
MSG6:            .byte "Hex conv. error",CR,0
MSG7:            .byte LF,"Poke: Fmt addr:dt",CR,0
MSG8:            .byte LF,"Copy block:  AddrFrom AddrTo Lenght(XXXX.XXXX:XXXX)",CR,0
MSG9:            .byte "Syntax error",CR,0
MSG10:           .byte "Turn on/off character echo",CR,0
MSG11:           .byte LF,"Fill block:  AddrFrom AddrTo data(XXXX.XXXX:XX)",CR,0

;;Help
HELP:            .byte CR,"Help for biosmon Version: ",VERSION,CR,LF
                 .byte "Commands:",CR
                 .byte "         B - memory Block copy",CR
                 .byte "         D - Dump memory",CR
                 .byte "         F - Fill memory",CR
                 .byte "         M - Memory poke",CR
                 .byte "         P - Peek memory",CR
                 .byte "         R - Run program",CR
                 .byte "         * - Turn on/off character echo",CR
                 .byte "         ? - Show help",CR,LF,0

;Used just for test of run cmd. 

OLD_WOZ:
                LDA     #<MSG5
                STA     MSGL
                LDA     #>MSG5
                STA     MSGH
                JSR     SHWMSG
                JMP     NEXT_CHAR

.include "drv16550.s"

.segment "RESETVEC"

                .word   $0F00          ; NMI vector
                .word   RESET          ; RESET vector
                .word   $0000          ; IRQ vector
