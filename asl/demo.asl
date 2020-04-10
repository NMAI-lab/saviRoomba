/**
 * @author	Chidiebere Onyedinma
 * @date	6 April 2020
 */

 /* Rules */

 lineCenter :-
        position(center).

 lineRight :-
        position(right).

 lineLeft :-
        position(left).

 lineAcross :-
        position(across).

 lineLost :-
        position(lost).

 /* Plans */

 !navigate.

 +!navigate
    : lineCenter
    <- drive(forward);
    !navigate.

 +!navigate
    : lineRight
    <- drive(right);
    !navigate.

 +!navigate
    : lineLeft
    <- drive(left);
    !navigate.

 +!navigate
    : lineAcross
    <- drive(stop);
    !navigate.

 +!navigate
    : lineLost
    <- drive(stop);
    !navigate.

 +!navigate.