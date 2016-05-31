/*  $Id$
 * 
 *  Copyright 2010 Anders Wallin (anders.e.e.wallin "at" gmail.com)
 *  
 *  This file is part of OpenCAMlib.
 *
 *  OpenCAMlib is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  OpenCAMlib is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with OpenCAMlib.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <string>
#include <iostream>
#include <sstream>

#include "clpoint.hpp"

namespace ocl
{
    
/* ********************************************** CLPoint *************/

CLPoint::CLPoint() 
    : Point() {
    cc = new CCPoint();
}

CLPoint::CLPoint(double x, double y, double z) 
    : Point(x,y,z) {
    cc = new CCPoint();
}

CLPoint::CLPoint(double x, double y, double z, CCPoint& ccp) 
    : Point(x,y,z) {
    cc = new CCPoint( ccp );
}


CLPoint::CLPoint(const CLPoint& cl) 
    : Point(cl.x,cl.y,cl.z) {
    cc = new CCPoint( *cl.cc );
}

CLPoint::CLPoint(const Point& p) 
    : Point(p.x,p.y,p.z) {
    cc = new CCPoint();
}

CLPoint::~CLPoint() {
   delete cc;
}

bool CLPoint::below(const Triangle& t) const {
    if (z < t.bb.maxpt.z )
        return true;
    else
        return false;
}

bool CLPoint::liftZ(const double zin) {
    if (zin>z) {
        z=zin;
        return true;
    } else {
        return false;
    }
}

bool CLPoint::liftZ(double zin, CCPoint& ccp) {
    if (zin>z) {
        z=zin;
        if (cc)
            delete cc;
        cc=new CCPoint( ccp );
        return true;
    } else {
        return false;
    }
}

bool CLPoint::liftZ_if_InsidePoints(double zin, CCPoint& cc_tmp, const Point& p1,const Point& p2) {
    if ( cc_tmp.isInside(p1, p2) ) 
        return this->liftZ(zin, cc_tmp);
    return false;
}

bool CLPoint::liftZ_if_inFacet(double zin, CCPoint& cc_tmp, const Triangle& t) {
    if ( cc_tmp.isInside(t) ) 
        return this->liftZ(zin, cc_tmp);
    return false;
}

CLPoint& CLPoint::operator=(const CLPoint &clp) {
    if (this == &clp)   // check for self-assignment
        return *this;
    x=clp.x;
    y=clp.y;
    z=clp.z;
    cc= new CCPoint( *(clp.cc) );
    return *this;
}

const CLPoint CLPoint::operator+(const CLPoint &p) const {
    return CLPoint(this->x + p.x, this->y + p.y, this->z + p.z);
}

const CLPoint CLPoint::operator+(const Point &p) const {
    return CLPoint(this->x + p.x, this->y + p.y, this->z + p.z);
}

CCPoint CLPoint::getCC() {
    return *cc;
}

std::string CLPoint::str() const {
    std::ostringstream o;
    o << "CL(" << x << ", " << y << ", " << z << ") cc=" << *cc ;
    return o.str();
}

} // end namespace
// end file clpoint.cpp
