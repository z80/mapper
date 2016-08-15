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
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string_regex.hpp>
#include <boost/filesystem.hpp>
#include <boost/regex.hpp>
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
    Node * left;
    Node * right;
    int data;

    const Node & operator=( const Node & inst )
    {
        left  = inst.left;
        right = inst.right;
        data  = inst.data;
        return *this;
    }
};

bool serializeTree( Node * root, std::string & stri )
{
  std::ostringstream out;
  out << root->data;
  if ( !root->left )
    out << ", NULL";
  else
  {
    out << ", ";
    std::string temp;
    serializeTree( root->left, temp );
    out << temp;
  }
  if ( !root->right )
    out << ", NULL";
  else
  {
    out << ", ";
    std::string temp;
    serializeTree( root->right, temp );
    out << temp;
  }
}

bool createTree( Node * & node, std::list<std::string> & lexems );

bool deserializeTree( const std::string & stri )
{
  // Read Value.
  boost::regex re( "([0-9A-Z]+)" );
  auto iterStart = boost::sregex_iterator( stri.begin(), stri.end(), re );
  std::list<std::string> lexems;
  for ( auto i=iterStart; i!= boost::sregex_iterator(); ++i )
  {
    boost::smatch match = *i;
    std::string str = match.str();
    lexems.push_back( str );
  }
  Node * root = 0;
  createTree( root, lexems );
}

bool createTree( Node * & node, std::list<std::string> & lexems )
{
  if ( lexems.size() < 1 )
    return false;
  boost::regex re( "[0-9]+" );
  boost::smatch match;
  std::string stri = lexems.front();
  lexems.pop_front();
  if( boost::regex_match( stri, match, re ) )
  {
    std::istringstream in( stri );
    int val;
    in >> val;
    Node * node = new Node();
    node->data = val;
    // Now parse left and right nodes.
    createTree( node->left,  lexems );
    createTree( node->right, lexems );
  }
}



int main()
{
  
  return 0;
}






