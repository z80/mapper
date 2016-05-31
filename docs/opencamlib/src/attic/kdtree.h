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

#ifndef KDTREE_H
#define KDTREE_H

#include <iostream>
#include <list>
#include "clpoint.h"

namespace ocl
{
    
class Point;
class Triangle;
class MillingCutter;

/// \brief KDTree spread, a measure of how spread-out a list of triangles are.
///
/// simple struct-like class for storing the "spread" or maximum 
/// extent of a list of triangles. Used by the kd-tree algorithm.
class Spread {
    public:
        /// constructor
        Spread(int dim, double v, double s);
        /// dimension
        int d;
        /// spread-value
        double val;
        /// minimum or start value
        double start;
        /// comparison of Spread objects. Used for finding the largest spread
        /// along which the next partition/cut is made.
        static bool spread_compare(Spread *x, Spread *y);
};

/// \brief K-D tree node. http://en.wikipedia.org/wiki/Kd-tree
///
/// A k-d tree is used for searching for triangles overlapping with the cutter.
///
/// Using a kd-tree with drop-cutter this is mentioned in a paper by Yau et al. 
/// http://dx.doi.org/10.1080/00207540410001671651
class KDNode {
    public:
        /// Create a node which partitions(cuts) along dimension d, at 
        /// cut value cv, with child-nodes hi_c and lo_c.
        /// If this is a bucket-node containing triangles, 
        /// they are in the list tris
        /// lev indicates the level of the node in the tree
        KDNode(int d, double cv, KDNode *up_c,
                                 KDNode *hi_c, 
                                 KDNode *lo_c, 
                                 const std::list<Triangle> *tlist, 
                                 int level);
        /// string repr
        std::string str() const;
        /// string repr
        friend std::ostream &operator<<(std::ostream &stream, const KDNode node);
        
        /// level of node in tree 
        int level;
        /// dimension of cut or partition.
        int dim;
        /// Cut or partition value.
        /// Child node hi should contain only triangles with a higher value than this.
        /// Child node lo contains triangles with lower values.
        double cutval;
        /// parent-node
        KDNode *up;
        /// Child-node hi.
        KDNode *hi;
        /// Child-node lo.
        KDNode *lo;
        /// A list of triangles, if this is a bucket-node
        const std::list<Triangle> *tris;
        
        
        /* static functions to build and search KD-trees) */
        /// build a kd-tree from a list of triangles. return root of tree.
        static KDNode* build_kdtree(const std::list<Triangle> *tris, 
                                    unsigned int bucketSize = 1,
                                    int level = 0,
                                    KDNode *parent=NULL);
        
        /// calculate along which dimension kd-tree should cut
        static Spread* spread(const std::list<Triangle> *tris);
        
        /// search KDTree, starting at KDNode root for triangles under the 
        /// MillingCutter c positioned at Point cl.
        /// The triangles found are pushed into the Triangle list tris.
        static void search_kdtree( std::list<Triangle> *tris, //why static?
                                   const CLPoint& cl, 
                                   const MillingCutter& cutter, 
                                   KDNode *root);
        
        /// find all triangles under node
        static void getTriangles( std::list<Triangle> *tris, 
                                   KDNode *root);
        
        /// do the triangles at KDNode root overlap (in the XY-plane) with the 
        /// MillingCutter c positioned at Point cl?
        static bool overlap(const KDNode *root,      // root of tree
                            const CLPoint& cl,       // cutter positioned here
                            const MillingCutter& c); // cutter
        
        /// counter used for cutting along the 4 dims sequentially.
        /// not used if cutting along max spread.
        static int cutcount;        
};

} // end namespace
#endif
// end file kdtree.h
