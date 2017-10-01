extern free
extern malloc
extern solver_set_bnd
extern solver_lin_solve

section .rodata
divMask: dd -0.5, -0.5, -0.5, -0.5
selectMask: dd 0x0000, 0x11111111, 0x11111111, 0x0000   
ceroComaCinco: dd 0.5, 0.5, 0.5, 0.5
negativeMask: dd -1, -1, -1, -1

cleanMask: dd 11111111111111111111111111111111b , 11111111111111111111111111111111b, 0x0, 0x0
cleanMask2: dd 0x0, 0x0, 11111111111111111111111111111111b , 11111111111111111111111111111111b

section .text

;typedef struct fluid_solver_t {
;    uint32_t N;       0          
;    float dt,          4   
;    float diff,       8  
;    float visc;        12       
;    float * u,         16 
     ;float* v,         24
     ;float * u_prev,
     ;float * v_prev; //velocity fields
;    float * dens, * dens_prev;  //density fields
;} __attribute__((__packed__)) fluid_solver;

%define offset_solver_n 0
%define offset_solver_u 16
%define offset_solver_v 24

global solver_lin_solve2


;void solver_lin_solve ( fluid_solver* solver, uint32_t b, float * x, float * x0, float a, float c );
;uint32_t i, j, k;
;   for ( k=0 ; k<20 ; k++ ) {
;       FOR_EACH_CELL
;           x[IX(i,j)] = (x0[IX(i,j)] + a*(x[IX(i-1,j)]+x[IX(i+1,j)]+x[IX(i,j-1)]+x[IX(i,j+1)]))/c;
;       END_FOR
;       solver_set_bnd ( solver, b, x );
;   


solver_lin_solve2:
;rdi = solver, esi = b, rdx = x, rcx = x0, xmm0 = a, xmm1 = c
    push rbp
    mov rbp, rsp
    push rbx
    push r12
    push r13
    push r14
    push r15
    sub rsp, 8

    xor r12, r12 
    mov r12d, [rdi + offset_solver_n] ;muevo a r12 el valor de N, que es un entero de 32 bits

    xor r13, r13 ;uso r13 para saber cuantas iteraciones debo hacer del algoritmo

    xor r8, r8
    xor r9, r9 ;uso r8 y r9 para guardar los punteros a cada matriz
    mov r8, rdx
    mov r9, rcx

    shufps xmm0, xmm0, 00000000b ;uso un shuffle para cuadruplicar el a para poder multiplicar varios resultados a la vez
                                ;xmm0 viejo = [0, 0, 0, a] -----> xmm0 nuevo = [a, a, a, a]
    shufps xmm1, xmm1, 00000000 ;idem para c, xmm1 viejo = [0, 0, 0, c] -----> xmm0 nuevo = [c, c, c, c]

    .ciclo_20_iteraciones:

        ;rdx es el puntero a la matriz x, voy a manejarme con tres punteros para ir agarrando los valores necesarios
        lea r14, [rdx + 4*r12 + 8] ; creo el puntero a la fila de arriba, sumandole N+2 posiciones, que miden 4 bytes cada una
        lea r15, [r14 + 4*r12 + 8] ; creo el puntero a la fila de arriba de esta, sumandole N+2 posiciones
        
        ;|(r15  |..     |..     |r15)   |       |...
        ;|(r14  |..     |..     |r14)   |       |... los cuatro floats que puedo levantar en un solo xmm al mismo tiempo para cada fila
        ;|(rdx  |..     |..     |rdx)   |       |... con respecto a la posicion del puntero

        lea rcx, [rcx + 4*r12 + 8] ;muevo el puntero a la matriz x0 a la misma posicion que esta r14 en la otra matriz

        ;|      |       |       |       |       |...
        ;|(rcx  |..     |..     |rcx)   |       |... los cuatro floats que puedo levantar en un solo xmm al mismo tiempo para cada fila
        ;|      |       |       |       |       |...

        
        mov rax, r12 ;uso rax para determinar el numero de columnas sobre las cuales iterar
            .ciclo_filas:
            mov rbx, r12 ;utilizo N para saber sobre cuantas columnas tengo que iterar

                .ciclo_columnas:
                movdqu xmm2, [rdx]
                movdqu xmm3, [r14]
                movdqu xmm4, [r15]

                movdqu xmm5, [rcx]

                movdqu xmm6, xmm3 ; hago una copia de los floats levantados de la fila del medio
                pslldq xmm3, 8    ; coloco en el medio los floats de la izquierda de los dos floats que estoy calculando 
                psrldq xmm3, 4    ; xmm3 viejo = [d, c, b, a] -----> xmm3 actual = [0, b, a, 0]

                psrldq xmm6, 8    ; coloco en el medio los floats de la derecha de los dos floats que estoy calculando 
                pslldq xmm6, 4    ; xmm6 viejo = [d, c, b, a] -----> xmm3 actual = [0, d, c, 0]
                addps xmm3, xmm6  ; sumo xmm3 + xmm6 para conseguir x(i+1,j) + x(i-1, j) de los dos floats a calcular 

                pslldq xmm2, 4    ; limpio los floats de los extremos que no me interesan al resultado actual
                psrldq xmm2, 8    ; xmm2 viejo = [d, c, b, a] -----> xmm2 actual = [0, c, b, 0]
                pslldq xmm2, 4
                addps xmm3, xmm2  ; le sumo los floats de abajo para que cada uno de los dos valores que estoy calculando quede  x(i+1,j) + x(i-1, j) + x(i, j-1)

                pslldq xmm4, 4    ; limpio los floats de los extremos que no me interesan al resultado actual
                psrldq xmm4, 8    ; xmm4 viejo = [d, c, b, a] -----> xmm4 actual = [0, c, b, 0]
                pslldq xmm4, 4
                addps xmm3, xmm4  ; le sumo los floats de arriba para que cada uno de los dos valores que estoy calculando quede  x(i+1,j) + x(i-1, j) + x(i, j-1) + x(i, j+1)

                mulps xmm3, xmm0  ; multiplico ambos valores por a

                pslldq xmm5, 4    ; limpio los floats de los extremos que no me interesan al resultado actual
                psrldq xmm5, 8
                pslldq xmm5, 4
                addps xmm3, xmm5  ; le sumo a mi resultado el valor que tenia en esa posicion en la matriz x0

                ;y luego divido por c
                divps xmm3, xmm1

                psrldq xmm3, 4    ; coloco el resultado en memoria
                movq [r14 + 4], xmm3 

                lea rdx, [rdx + 8]
                lea r14, [r14 + 8]
                lea r15, [r15 + 8]
                lea rcx, [rcx + 8]
                sub rbx, 2
                cmp rbx, 0
                jne .ciclo_columnas

            dec rax
            lea rdx, [rdx + 8]
            lea r14, [r14 + 8]
            lea r15, [r15 + 8]
            lea rcx, [rcx + 8]

            cmp rax, 0
            jne .ciclo_filas

        mov rdx, r8
        mov rcx, r9 ;vuelvo a dejar los punteros a las matrices en los principios de estas para la siguiente iteracion de las 20 del algoritmo
        inc r13
        cmp r13, 20
        jne .ciclo_20_iteraciones

    call solver_set_bnd
             
    add rsp, 8
    pop r15
    pop r14
    pop r13
    pop r12
    pop rbx
    pop rbp
ret


; void solver_set_bnd ( fluid_solver* solver, uint32_t b, float * x ){
; 	uint32_t i;
; 	uint32_t N = solver->N;
; 	for ( i=1 ; i<=N ; i++ ) {
; 		x[IX(0  ,i)] = b==1 ? -x[IX(1,i)] : x[IX(1,i)];
; 		x[IX(N+1,i)] = b==1 ? -x[IX(N,i)] : x[IX(N,i)];
; 		x[IX(i,0  )] = b==2 ? -x[IX(i,1)] : x[IX(i,1)];
; 		x[IX(i,N+1)] = b==2 ? -x[IX(i,N)] : x[IX(i,N)];
; 	}
; 	x[IX(0  ,0  )] = 0.5f*(x[IX(1,0  )]+x[IX(0  ,1)]);
; 	x[IX(0  ,N+1)] = 0.5f*(x[IX(1,N+1)]+x[IX(0  ,N)]);
; 	x[IX(N+1,0  )] = 0.5f*(x[IX(N,0  )]+x[IX(N+1,1)]);
; 	x[IX(N+1,N+1)] = 0.5f*(x[IX(N,N+1)]+x[IX(N+1,N)]);
; }



global solver_set_bnd2
solver_set_bnd2:
    ;rdi = solver
    ;rsi = b 
    ; push rbp
    ; mov rbp, rsp
    ; push rbx
    ; push r12
    ; push r13
    ; push r14
    ; push r15
    ; sub rsp, 8


    ; .code:
    ;     mov rbx, rdx ; rbx == *x
    ;     add rbx, 4 ; rbx += 4
        
    ;     mov r12, [rdi] ; r12 = N
    ;     mov r10, [rdi] ; r10 = N
        
    ;     add r10, 8; r10 = N + 2

    ;     add r12, [rdi] ; r12 = 2N
    ;     add r12, 4 ; r12 = 2N + 1
    ;     add r12, 8 ; r12 = 2N + 1 + 2


    ; .loop_up_down:    

    ;     mov r8, rsi ; r8 = b
    ;     sub r8, 1 ; r8 = b - 1
    ;     je .bEsDos ; r8 == 0

    ;     .bNoEsDos:

    ;         xor r15, r15 ; r15 = 0
    ;         add r15, rbx ; r15 = i
    ;         add r15, r10 ; r15 = N + 2 + i
    ;         movdqu xmm1, [r15] ; xmm1 = [x[i,1] | x[i+1,1] | x[i+2,1] | x[i+3,1]]
    ;         movdqu [rbx], xmm1 ; pongo en x[i,0]
            
    ;         xor r15, r15 
    ;         mov r14, r10 ; r14 = N + 2
    ;         sub r14, 8 ; r14 = N
    ;         mov r15, r10 ; r15 = N
    ;         mul r15, r14 ; r15 = (N + 2) * N
    ;         add r15, rbx ; r15 = (N + 2) * N + i
    ;         ; x[IX(N+1,i)] = x[IX(N,i)];
    ;         movdqu xmm1, [r15] ; xmm1 = [x[i, N] | x[i+1, N] | x[i+2, N] | x[i+3, N]]
            
    ;         add r14, 8 ; r14 = N + 1
    ;         add r15, r14 ; r15 = (N + 2) * (N + 1) + i

    ;         movdqu [r15], xmm1 ; pongo en x[i, N+1]

    ;         jmp .counters



    ;     .bEsDos:
    ;         ; x[IX(i,0)] = x[IX(i,1)];
    ;         ; r10 = N+2
    ;         ; rbx = i
            
    ;         xor r15, r15 ; r15 = 0
    ;         add r15, rbx ; r15 = i
    ;         add r15, r10 ; r15 = N + 2 + i
    ;         movdqu xmm1, [r15] ; xmm1 = [x[i,1] | x[i+1,1] | x[i+2,1] | x[i+3,1]]
    ;         movdqu xmm7, [negativeMask] ; xmm7 = [-1 | -1 | -1 | -1]
    ;         mulss xmm1, xmm7 ; xmm7 = [-x[i,1] | -x[i+1,1] | -x[i+2,1] | -x[i+3,1]]
    ;         movdqu [rbx], xmm1 ; pongo en x[i,0]


            
    ;         xor r15, r15 
    ;         mov r14, r10 ; r14 = N + 2
    ;         sub r14, 8 ; r14 = N
    ;         mov r15, r10 ; r15 = N
    ;         mul r15, r14 ; r15 = (N + 2) * N
    ;         add r15, rbx ; r15 = (N + 2) * N + i

    ;         ; x[IX(i,N+1)] = x[IX(i,N)];
    ;         movdqu xmm1, [r15] ; xmm1 = [x[N,i] | x[N,i+1] | x[N,i+2] | x[N,i+3]]
            
    ;         add r14, 8 ; r14 = N + 1
    ;         add r15, r14 ; r15 = (N + 2) * (N + 1) + i
            
    ;         movdqu xmm7, [negativeMask] ; xmm7 = [-1 | -1 | -1 | -1]
    ;         mulss xmm1, xmm7 ; xmm7 = [-x[1,i] | -x[1,i+1] | -x[1,i+2] | -x[1,i+3]]

    ;         movdqu [r15], xmm1 ; pongo en x[N+1,i]



    ;     add rbx, 16
    ;     add r12, 16
    ;     jmp .loop_up_down




    ;    ; ---------------------------
    ; .loop_der_izq:
        
        


    ;     mov rbx, [rdi]
    ;     add rbx, 1

    ;     mov r14, [rdi]
    ;     add r14, 1 ; r14 = N + 1
                
    ;     mov r11, [rdi]
    ;     mul r11, 4 ; r11 = N * 4
        

    ;     mov r8, rsi ; r8 = b
    ;     sub r8, 1 ; r8 = b - 1
    ;     cmp r8, 0

    ;     je .bEsUno_der_izq ; r8 == 0
        

    ;     .bNoEsUno_der_izq:

    ;         mov r13, rdx 
    ;         add r13, r11
            
    ;         movdqu xmm1, [r13]
    ;         movdqu xmm2, xmm1 
    ;         shufps xmm1, xmm2, 11110000b  

    ;         movdqu xmm7, [negativeMask] ; xmm7 = [-1 | -1 | -1 | -1]
    ;         mulss xmm1, xmm7 ; xmm7 = [-x[1,i] | -x[1,i+1] | -x[1,i+2] | -x[1,i+3]]

    ;         movdqu [r13], xmm1

    ;         sub rbx, 1
    ;         cmp rbx, 0
    ;         je .fillBorders

    ;         jmp .loop_der_izq    


    ;     .bEsUno_der_izq:

    ;         mov r13, rdx 
    ;         add r13, r11
            

    ;         movdqu xmm1, [r13]
    ;         movdqu xmm2, xmm1 
    ;         shufps xmm1, xmm2, 11110000b  

    ;         movdqu [r13], xmm1

    ;         sub rbx, 1
    ;         cmp rbx, 0
    ;         je .fillBorders  

    ;         jmp .loop_der_izq    
            



    ; .fillBorders:
    ;     ;[0,0]

    ;     mov r10, [rdi]
    ;     mul r10, 4

    ;     mov rcx, rdx 
    ;     mov rbx, rdx
    ;     add rbx, 4
    ;     movss xmm2, [rbx]
    ;     addss xmm1, xmm2
    ;     movdqu xmm7, [ceroComaCinco]
    ;     mulss xmm1, xmm7
    ;     movss [rbx], xmm1

    ;     add rdx, r10 ; rbx += 4*N
    ;     add rdx, 4 ; rbx += 4

    ;     ;[0, N+1]
    ;     mov rbx, rdx
    ;     sub rbx, 4
    ;     movss xmm1, [rbx]
    ;     add rbx, r10 ; rbx += 4*N
    ;     add rbx, 12; rbx += 12 
    ;     movss xmm2, [rbx]
    ;     addss xmm1, xmm2
    ;     movdqu xmm7, [ceroComaCinco]
    ;     mulss xmm1, xmm7
    ;     movss [rdx], xmm1

    ;     ;[N+1, 0]
    ;     mov rdx, rcx
    
    ;     mov r11, r10
    ;     mov r12, r10
    ;     add r11, 2 
    ;     add r12, 1
    ;     mul r11, r12
    ;     mul r11, 4

    ;     add rdx, r11
    ;     mov rbx, rdx
    ;     add rbx, 4
    ;     movss xmm1, [rbx]
    ;     sub rbx, r10
    ;     sub rbx, 12
    ;     movss xmm2, [rbx]
    ;     addss xmm1, xmm2
    ;     movdqu xmm7, [ceroComaCinco]        
    ;     mulss xmm1, xmm7

    ;     ; [N+1, N+1]
    ;     movss [rdx], xmm1
    ;     add rdx, r10 ; rbx += 4*N
    ;     add rdx, 4 ; rbx += 4
    ;     mov rbx, rdx
    ;     sub rbx, 4
    ;     movss xmm1, [rbx]
    ;     sub rbx, r10
    ;     sub rbx, 4
    ;     movss xmm2, [rbx]
    ;     addss xmm1, xmm2
    ;     movdqu xmm7, [ceroComaCinco]        
    ;     mulss xmm1, xmm7
    ;     movss [rbx], xmm1 





    ; add rsp, 8
    ; pop r15
    ; pop r14
    ; pop r13
    ; pop r12
    ; pop rbx
    ; pop rbp

ret




global solver_project
solver_project:
    ;solver_project_first ( solver, p, div  );
    ; RDI , RSI, RDX
    push rbp
    mov rbp, rsp
    push rbx
    push r12
    push r13
    push r14
    push r15


    ;FOR_EACH_CELL
    ;    div[IX(i,j)] = -0.5f*(solver->u[IX(i+1,j)]-solver->u[IX(i-1,j)]+solver->v[IX(i,j+1)]-solver->v[IX(i,j-1)])/solver->N;
    ;    p[IX(i,j)] = 0;
    ;END_FOR 

    mov R12, RDI ; Solver 
    mov R13, RSI ; P
    mov RBX , RDX ; Div

    xor R8, R8
    mov R8d , [ RDI ] ; Guardo N

    mov R9, R8  ; R9 = N
    shl R9d, 2  ; R9 = (N * 4)
    add R9, 8   ; R9 = (N+2) * 4 - Lo usaremos para saltar de fila en fila

    mov RAX, R8;
    shr RAX, 1 ; Lo divido por dos que van a ser los movimientos que hay que hacer por fila

    mov RCX, R8; Le muevo N, sera el contador de filas  

    ; Acomodo div
    add RDX , R9 ; Lo pongo en la segunda fila
    add RDX , 4  ; Avanzo al primer elemento de la misma

    ; Obtengo p
    ; Lo tengo en RSI
    add RSI , R9 ; Lo pongo en la segunda fila
    add RSI , 4  ; Lo avanzo al segundo elemento de la segunda fila

    pxor XMM7, XMM7 ; Lo utilizare para poner 0s

    ; Obtener U
    mov R10 , [ RDI + offset_solver_u ] 
    add R10 , R9
    ;add R10 , 4 ; Lo avanzo a la segunda fila y evito el borde

    ; Obtener V 
    mov R14 , [ RDI + offset_solver_v ]
    ;add R14 , 4 ; Lo avanzo al segundo elemento 

    mov R15 , R14 ; Lo copio, este va ir a la tercera fila

    ; Los tres apuntan al inicio de sus respectivas matrices 
    ; add R10 , R9 ; El de U debe apuntar a la primer fila no borde
    add R15 , R9
    add R15 , R9 ; Pongo R15 en la tercera fila

    ; Creo una mascara de todos n
    pxor XMM8, XMM8
    cvtsi2ss XMM8, R8 ; XMM8 = [ n ! 0 ! 0 ! 0 ]
    shufps XMM8, XMM8, 00000000 ; XMM8 = [ n ! n ! n ! n ]


    .loop:
     pxor XMM0, XMM0    ; XMM0 = [  0 |  0 |  0 |  0 ]
     pxor XMM5, XMM5  

     movdqu XMM1 ,[R10] ; XMM1 = [ u3 | u2 | u1 | u0 ]
     movdqu XMM2 , XMM1 ; XMM2 = [ u3 | u2 | u1 | u0 ]

     pslldq XMM1 , 4    ; XMM1 = [ u2 | u1 | u0 | 0  ]
     psrldq XMM2 , 4    ; XMM2 = [ 0  | u3 | u2 | u1 ]

     movdqu XMM0 , XMM2  ; XMM0 = [ 0  | u3    | u2      |  u1  ]
     subps XMM0 , XMM1  ; XMM0 = [ -  | u3-u1 | u2 - u0 |   -  ]

     movdqu XMM3 ,[R14] ; XMM3 = [ v03 | v02 | v01 | v00 ]
     movdqu XMM4 ,[R15] ; XMM4 = [ v23 | v22 | v21 | v20 ]

     subps XMM0 , XMM3
     addps XMM0 , XMM4 ; XMM0 = [ - | u3-u1 + v22 - v02 | u2 - u0 + v21 - v01 | - ]

     divps XMM0, XMM8  ; XMM0 = [ - | (u3-u1 + v12 - v02) / n | (u2 - u0 + v11 - v01) / n | - ]

     movdqu XMM5, [divMask]  ; Mascara - XMM5 = [ -0.5 | -0.5 | -0.5 | -0.5 ]
     mulps XMM0, XMM5  ; XMM0 = [ - | -0.5 * ( (u3-u1 + v12 - v02) / n ) | -0.5 *  ( (u2 - u0 + v11 - v01) / n ) | - ]
     psrldq XMM0, 4    ; XMM0 = [ 0 | - | -0.5 * ( (u3-u1 + v12 - v02) / n ) | -0.5 *  ( (u2 - u0 + v11 - v01) / n ) ];

     movdqu XMM5, [cleanMask] ; Mascara - XMM5 = [ 0 | 0 | 1 | 1 ]
     pand  XMM0, XMM5  ; XMM0 = [ 0 | -0.5 * ( (u3-u1 + v12 - v02) / n ) | -0.5 *  ( (u2 - u0 + v11 - v01) / n ) | 0 ]

     movdqu XMM5, [cleanMask2]
     movdqu XMM1, [RDX] ; Agarro los valores que tenia en div para no pisar los que quiero (el primero) XMM1 = [ div3 | div2 | div1 | div0 ]
     pand XMM1, XMM5  ; XMM1 = [ div3 | div2 | 0 | 0 ]
     addps XMM0, XMM1  ; XMM0 = [ div3 | div2 | -0.5 * ( (u3-u1 + v12 - v02) / n ) | -0.5 *  ( (u2 - u0 + v11 - v01) / n )]
     ;movdqu [0x0], XMM0
     movdqu [RDX] , XMM0 ; Muevo los valores modificados a memoria
     movdqu [RSI] , XMM7 ; Pongo 0 en las posiciones de P

     ;jmp .fin; BORRAR
     dec RAX   ; Decremento el contador de movimientos de la fila
     cmp RAX, 0 ; Si termino de completar los elementos de la fila
     jne .continuarFila
     dec RCX    ; Resto uno al contador de filas
     cmp RCX, 0
     je .finP1
     
     ;jmp .fin; BORRAR
     ;Reinicio contador de movimientos de la fila
     mov RAX, R8;
     shr RAX, 1 

     add RDX, 16  ; Lo mando a la siguiente fila (4 * 4)
     add R15, 16 ; Lo mando a la siguiente fila
     add R14, 16 ; Lo mando a la siguiente fila
     add RSI, 16 ;
     add R10, 16 ; Lo mando a la siguiente fila
     jmp .loop

     .continuarFila:
     add RDX, 8 ; Lo mando a los siguientes dos elementos
     add R15, 8 ; Lo mando a los siguientes dos elementos
     add R14, 8 ; Lo mando a los siguientes dos elementos
     add RSI, 8 ; Lo mando a los siguientes dos elementos
     add R10, 8 ; Lo mando a los siguientes dos elementos
     jmp .loop


     ; FINALIZADO PRIMER FOR
    .finP1:

        ;solver_set_bnd ( solver, 0, div )
        mov RDI, R12 ; solver
        mov ESI, 0x0 ; 0
        mov RDX, RBX ; div
        call solver_set_bnd

        ;solver_set_bnd ( solver, 0, p );
        mov RDI, R12 ; solver
        mov ESI, 0x0 ; 0
        mov RDX, R13 ; p
        call solver_set_bnd

        ;solver_lin_solve ( solver, 0, p, div, 1, 4 );
        ;void solver_lin_solve ( fluid_solver* solver, uint32_t b, float * x, float * x0, float a, float c );
        mov RDI, R12 ; 
        mov ESI, 0 ; 
        mov RDX, R13 ; 
        mov RCX, RBX  ;
        mov R8d, 1;
        mov R9d, 4;
        cvtsi2ss XMM0, R8d
        cvtsi2ss XMM1, R9d ;
        call solver_lin_solve

        

        ;FOR_EACH_CELL
        ;     solver->u[IX(i,j)] -= 0.5f*solver->N*(p[IX(i+1,j)]-p[IX(i-1,j)]);
        ;     solver->v[IX(i,j)] -= 0.5f*solver->N*(p[IX(i,j+1)]-p[IX(i,j-1)]);
        ;END_FOR

        xor R8, R8
        xor R9, R9
        xor R10, R10

        mov R8d, [ R12 + offset_solver_n ]; N
        mov R9d , R8d

        shr R9d, 1 ; N/2
        mov R10d, R9d; 

        mov R11 , R13 ; El puntero a p
        mov R14 , [ R12 + offset_solver_u ]
        add R14, 4; Tamaño de Float, avanzo uno
         ; Avanzo a la fila siguiente?

        pxor XMM2, XMM2
        mov R15, R9 ; n * 0.5
        cvtsi2ss XMM2, R15      ; XMM2 = [ n * 0.5 ! 0 ! 0 ! 0 ]
        shufps XMM2, XMM2, 00000000 ; XMM2 = [ n * 0.5 ! n * 0.5 ! n * 0.5 ! n * 0.5 ]

            ;solver->u[IX(i,j)] -= 0.5f*solver->N*(p[IX(i+1,j)]-p[IX(i-1,j)]);

            .loops:
             movdqu XMM0, [ R11 ]; XMM0 = [ p3 ! p2 ! p1 ! p0 ]
             movdqu XMM1, XMM0   ; XMM1 = [ p3 ! p2 ! p1 ! p0 ]
             psrldq XMM1, 8      ; XMM1 = [  0 !  0 ! p3 ! p2 ]
             subps XMM1, XMM0      ; XMM1 = [  - !  - ! p3 - p1 ! p2 - p0 ]

             mulps XMM1, XMM2    ; XMM1 = [ - ! - ! n * 0.5 * (p3 - p1) ! n * 0.5 * (p3 - p0)]

             movdqu XMM4, [R14]  ; XMM4 = [ u3 ! u2 ! u1 ! u0 ]

             movdqu XMM6, [cleanMask]; XMM6 = [ 0  !  0 ! 1  !  1 ]
             pand XMM1, XMM6     ; XMM1 = [  0 !  0 !  n * 0.5 * (p3 - p1) ! n * 0.5 * (p3 - p0) ]
             subps XMM4, XMM1    ; XMM4 = [ u3 ! u2 ! u1 - ( n * 0.5 * (p3 - p1) ) ! u0 - ( n * 0.5 * (p3 - p0) ) ]
             movdqu [R14], XMM4  ;
            
             add R11, 8      ; Avanzo dos posiciones ( 2 * 4 de Float) el puntero de p
             add R14, 8      ; Avanzo dos posiciones ( 2 * 4 de Float) el puntero de u 

             dec R10d      ; Le resto al contador de movimeintos de fila (2 elementos ya hechos)
             cmp R10d, 0
             je .nuevaFila
             jmp .loops

             .nuevaFila:
             dec R8d
             mov R10d, R9d     ; Cantidad de movimientos que tengo que hacer sobre puntero a p
             add R11, 8      ; Termino de saltar a la fila siguiente ( 2 * 4 de Float)
             add R14, 8      ; Termino de saltar a la fila siguiente ( 2 * 4 de Float)
             cmp R8d, 0       ; Si se terminaron las filas, termina
             je .finP2
             jmp .loops

    .finP2:

            ;solver->v[IX(i,j)] -= 0.5f*solver->N*(p[IX(i,j+1)]-p[IX(i,j-1)]);
            
            xor R8, R8
            mov R8d, [ R12 + offset_solver_n ]
            mov R9, R8   ; R9 = N
            mov R10 , R8 ; R10 = N

            shl R8, 2   ; R8 = ( N * 4 )   
            add R8, 8   ; R8 = ( N * 4 ) + ( 2 * 4 )
            shr R9, 2   ; R9 = ( N / 4 )   

            mov RCX, R9 ; Contador fila   
            mov R11, R9 ; Guardo este valor

            mov R15, [ R12 + offset_solver_v ] 
            add R15, 4   ; Salteo el borde (el primer float)

            mov RAX, R13 ; Lo situo en la matriz p
            add RAX, 4   ; Avanzo al siguiente de la matriz P para evitar el borde  
            mov R14, RAX ; Este lo voy a dejar abajo ( j - 1 )

            add R15, R8  ; P2 sera el central, que se situara en V ( j )

            add RAX, R8  
            add RAX, R8  ; Situo este puntero "dos filas" mas arriba (j+1)

        .ciclo:
            movdqu XMM0, [RAX]  ; XMM0 = [ p3-3 ! p3-2 ! p3-1 ! p3-0 ] 
            movdqu XMM1, [R14]  ; XMM1 = [ p1-3 ! p1-2 ! p1-1 ! p1-0 ]  
            subps  XMM0, XMM1   ; XMM0 = [ (p3-3) - (p1-3) ! (p3-2) - (p1-2) ! (p3-1) - (p1-1) ! (p3-0) - (p1-0) ]

            mulps XMM0, XMM2     ; XMM0 = [ n * .5 * ((p3-3) - (p1-3)) ! n * .5 * ( (p3-2) - (p1-2) ) ! n * .5 * ( (p3-1) - (p1-1) ) ! n * .5 * ( (p3-0) - (p1-0) ) ]

            movdqu XMM5, [ R15 ] ; Leo los cuatro valores de la fila a modificar
            subps XMM5, XMM0     ; Le resto los valores correspondientes
            movdqu [ R15 ], XMM5 ; Lo devuelvo a las filas

            dec RCX    ; Resto a la cantidad de movimientos por filas
            cmp RCX, 0 ; Si no tengo que hacer mas movimientos para leer por fila
            je .nuevaFila2

            add R14, 16      ; Avanzo a los siguientes 4 floats (4 * 4 floats)
            add R15, 16      ; Avanzo a los siguientes 4 floats (4 * 4 floats)
            add RAX, 16      ; Avanzo a los siguientes 4 floats (4 * 4 floats)
            jmp .ciclo

        .nuevaFila2:
            mov RCX, R11    ; n / 4, que es la cantidad de veces que voy a leer de a 4 floats por fila
            dec R10         ; Le resto al contador de la fila
            cmp R10, 0      ; Me fijo si ya termine con las filas
            je .fin

            add R14, 24      ; Avanzo los siguientes 4 floats y 2 mas para pasarlo del limite a la nueva fila 
            add R15, 24      ; Avanzo los siguientes 4 floats y 2 mas para pasarlo del limite a la nueva fila 
            add RAX, 24      ; Avanzo los siguientes 4 floats y 2 mas para pasarlo del limite a la nueva fila 
            jmp .ciclo

    .fin: 
        ;FINALIZADO SEGUNDO FOR

        ;solver_set_bnd ( solver, 1, solver->u )
        mov RDI, R12 ; solver
        mov ESI, 1h ; 0
        mov RDX, RBX ; div
        call solver_set_bnd

        ;solver_set_bnd ( solver, 2, solver->v );
        mov RDI, R12 ; solver
        mov ESI, 2h ; 0
        mov RDX, R13 ; p
        call solver_set_bnd

    pop r15
    pop r14
    pop r13
    pop r12
    pop rbx
    pop rbp
ret