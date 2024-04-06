;Driver for P8255A

PORTA = $B100
PORTB = $B101
PORTC = $B102
DDR   = $B103


INIT8255:
    lda $80
    sta DDR
    rts


    
