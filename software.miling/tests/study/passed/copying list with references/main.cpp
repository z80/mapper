#include <stdio.h>
//#include <conio.h>
#include <math.h>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <iostream>
#include <string>
#include <list>
#include <sstream>
#include <iostream>
#include <iomanip>
//#include <boost/algorithm/string.hpp>
//#include <boost/algorithm/string_regex.hpp>
//#include <boost/filesystem.hpp>
#include <regex>
#include <sstream>
#include <locale>
#include <iostream>
#include <queue>
#include <map>
//using namespace boost::filesystem;


// Copy list with references.
struct Node
{
    Node * next;
    Node * ref;
    int data;

    const Node & operator=( const Node & inst )
    {
        next = inst.next;
        ref  = inst.ref;
        data = inst.data;
        return *this;
    }
};

// Use correspondence list.
// The easiest way would be to use std::map<Node *, Node *>.

bool copyList( Node * head, Node * & result )
{
    // Copy as is and fill correspondence map.
    std::map<Node *, Node *> corr;
    Node * i = head;
    while ( i )
    {
        Node * newNode = new Node();
        newNode = i;
        i = i->next;
        corr[i] = newNode;
    }
    corr[0] = 0;

    // Set links.
    i = head;
    while ( i )
    {
        Node * newNode = corr[i];
        newNode->next = corr[i]->next;
        newNode->ref  = corr[i]->ref;
        i = i->next;
    }
    result = corr[head];
    return true;
}


int main()
{
    
    return 0;
}






