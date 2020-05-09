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
        postPoint(C,P) &
        (N > C).

 destBehind :-
        dest(N) &
        postPoint(C,P) &
        (N < C).

 atDestination :-
        dest(N) &
        postPoint(C,P) &
        (N = C).

 onTrack :-
        postPoint(_,P) &
        dest(N) &
        (((N > P) & destAhead) | ((N < P) & destBehind))) &
        lineCenter.

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

 +!deliver
    : atDestination
    <- drive(stop).

 +!deliver
    : batteryLow
    <- !dock.

 +!deliver
    : batteryOK & docked
    <- !undock.

 +!deliver.

 +!dock
    : not atDockPost & onTrack
    <- !navigate;
    !dock.

 +!dock
    : atDockPost & moving
    <- drive(stop);
    !dock.

 +!dock
    : atDockPost & not moving
    <- dock_bot.

 +!dock.

