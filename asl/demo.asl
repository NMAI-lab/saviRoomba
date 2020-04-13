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

 destAhead :-
        dest(N) &
        postPoint(C) &
        (N > C).

 onTrack :-
        postPoint() &
        lineCenter.

 destBehind :-
        dest(N) &
        postPoint(C) &
        (N < C).

 atDestination :-
        dest(N) &
        postPoint(C) &
        (N == C).

 /* Plans */

 !navigate.
 !deliver.

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

+!deliver
    : onTrack
    <- !navigate.

 +!navigate.