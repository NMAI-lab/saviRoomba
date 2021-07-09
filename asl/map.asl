/**
 * Map
 * a -- b -- c
 *		|
 *		|
 *		d
 */

// Possible routes between locations
possible(a,b).
possible(b,a).

possible(b,c).
possible(c,b).

possible(b,d).
possible(d,b).


// Location definitions
locationName(a,[1.34,0.25]).
locationName(b,[1.34,0]).
locationName(c,[1.34,-0.65]).
locationName(d,[0,0]).	// Robot has to start here, facing b.


// Successor state
suc(Current,Next,Range,drive)
	:-	possible(Current,Next)
		& range(Current,Next,Range).
	
// Heutistic definition: h(CurrentState,Goal,H)
h(Current,Goal,Range) 
	:-	range(Current,Goal,Range).
					
// Range
rangeName(A,B,Range)
	:-	locationName(A,[X1,Y1])
		& locationName(B,[X2,Y2])
		& range(X1,Y1,X2,Y2,Range).
		
// Range
range(X1,Y1,X2,Y2,Range)
	:-	Range = math.sqrt( ((X2-X1) * (X2-X1)) + ((Y2-Y1) * (Y2-Y1)) ).

// Position Rule
position(X,Y)
	:-	odomPosition(X,Y,_).

// Get name and range of nearest location
nearestLocation(Current,Range)
	:-	position(X,Y)
		& locationName(Current,[Xcurrent,Ycurrent])
		& locationName(Other,[Xother,Yother])
		& range(X,Y,Xcurrent,Ycurrent,Range)
		& range(X,Y,Xother,Yother,OtherRange)
		& Other \== Current
		& Range < OtherRange.

// Identify location of robot, if at a named location
atLocation(Location,Range)
    :-  nearestLocation(Location,Range)
		& Range < 0.1.
		
// Identify location of robot, if at a named location
nearLocation(Location,Range)
    :-  position(X,Y)
		& locationName(Location,[X,Y])
		& (not atLocation(Location,Range))
		& Range < 0.3.

