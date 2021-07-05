
/**
 * Definition of the map
 */
 
/**

locationName(a,[1,1]).
locationName(b,[1,0]).
locationName(c,[2,0]).
locationName(d,[0,0]).

// Possible map transitions.
// possible(StartingPosition, PossibleNewPosition)
possible(a,b).
possible(b,a).

possible(b,c).
possible(c,b).

possible(b,d).
possible(d,b).

beaconName(fce22e629b3d,a).
beaconName(ea2f93a69820,b).
beaconName(b827ebbd1009,d).
beaconName(b827eb7cff08,e).
*/

/**
 * PG's Map
 * c -- f -- g -- h
 */
 
locationName(c,[1,0]).
locationName(f,[2,0]).
locationName(g,[3,0]).
locationName(h,[4,0]).

possible(c,f).
possible(f,c).

possible(f,g).
possible(g,f).

possible(g,h).
possible(h,g).

beaconName(e277fcf90493,c).
beaconName(d06ad20242eb,f).
beaconName(ee16869ac2a8,g).
beaconName(e487913d1ed7,h).


atLocation(Location,Range)
    :-  beacon(Mac,Range)
		& beaconName(Mac,Location)
		& Range < 0.5.
		
nearestLocation(Current,Range)
	:-	beacon(Mac,Range)
		& beacon(_,OtherRange)
		& (not (OtherRange < Range))
		& beaconName(Mac,Current).

// Position Rule
position(X,Y)
	:-	nearestLocation(Current,_)
		& locationName(Current,[X,Y]).

// Successor state
suc(Current,Next,Range,drive)
	:-	possible(Current,Next)
		& range(Current,Next,Range).
	
// Heutistic definition: h(CurrentState,Goal,H)
h(Current,Goal,Range) 
	:-	range(Current,Goal,Range).
					
// Range
range(A,B,Range)
	:-	locationName(A,[X1,Y1])
		& locationName(B,[X2,Y2])
		& Range = math.sqrt( ((X2-X1) * (X2-X1)) + ((Y2-Y1) * (Y2-Y1)) ).
