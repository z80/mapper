#include <stdio.h>
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
#include <regex>
using namespace boost::filesystem;




int main()
{
    try {
        boost::regex re( "\\d{3}\\-?\\d{4}" );
        //std::regex re( "[0-9]{3}", std::regex::icase );
        //boost::regex re( "[a-z]" );
        boost::smatch match;
        std::string stri = "345-4323 fsdfs fsd 43243432 345-5456";
        if ( boost::regex_search( stri, match, re ) )
        {
            if ( match.size() > 0 )
                std::cout << "match: '" << match[0].str() << "'" << std::endl;
        }
        std::cout << "match.size() = " << match.size() << std::endl;

        // Search for all occurences.
        boost::sregex_iterator next( stri.begin(), stri.end(), re );
        while ( next != boost::sregex_iterator() )
        {
            std::cout << *next << std::endl;
            next++;
        }
    }
    catch ( const boost::regex_error & e )
    {
        std::cout << "regex_error caught: " << e.what() << ", code: " << e.code() << std::endl;
    }
    catch ( ... )
    {
        std::cout << "Error: escape characters." << std::endl;
    }












    path p ("./");   // p reads clearer than argv[1] in the following code

    std::string stri = canonical( p ).string();
    try
    {
        if (exists(p))    // does p actually exist?
        {
          if (is_regular_file(p))        // is p a regular file?
            std::cout << p << " size is " << file_size(p) << '\n';

          else if (is_directory(p))      // is p a directory?
          {
            std::cout << p << " is a directory containing:\n";

            //std::copy(directory_iterator(p), directory_iterator(),      // directory_iterator::value_type
            //  std::ostream_iterator<directory_entry>(std::cout, "\n")); // is directory_entry, which is
                                                                        // path stream inserter
            for ( auto i=p.begin(); i!=p.end(); i++)
            {
                path e = *i;
                for ( auto i=e.begin(); i!=e.end(); i++)
                {
                    std::cout << *i << std::endl;
                }
            }
          }

          else
            std::cout << p << " exists, but is neither a regular file nor a directory\n";
        }
        else
          std::cout << p << " does not exist\n";
    }

    catch (const filesystem_error& ex)
    {
        std::cout << ex.what() << '\n';
    }

    return 0;
}



