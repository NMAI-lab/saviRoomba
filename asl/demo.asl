/**
 * Demo BDI program
 * This example BDI program performs the action
 * 'do(action)' whenever it receives a perception 
 * of the format 'time(1234)'.
 * @author	Patrick Gavigan
 * @date	6 December 2019
 */

 /* Rules */

 lineCenter :-
        position(center).

 lineRight :-
        position(right).

 lineLeft :-
        position(left).

 lineAcross :-
        position(true,true,true).

 lineLost :-
        position(false,false,false).

 /* Plans */

 +!navigate
    : lineCenter
    <- drive(forward).

 +!navigate
    : lineRight
    <- drive(right).

 +!navigate
    : lineLeft
    <- drive(left).

 +!navigate
    : lineAcross
    <- drive(stop).

 +!navigate
    : lineLost
    <- drive(stop).

 +!navigate.