
#include "points_file.h"
#include <regex>

PointsFile::PointsFile( const std::string & fname )
{
    this->fname = fname;
}

PointsFile::~PointsFile()
{
}

bool PointsFile::write( const std::vector<cv::Point3f> & ats, const std::vector<cv::Point3f> & froms )
{
    out.open( fname );
    bool res = out.is_open();
    if ( !res )
        return false;
    out.seekp( 0, out.end );
    int sz = static_cast<int>( ats.size() );
    for ( int i=0; i<sz; i++ )
    {
        const cv::Point3f & at = ats[i];
        const cv::Point3f & from = froms[i];
        out << at.x << ", " << at.y << ", " << at.z << ", " << from.x << ", " << from.y << ", " << from.z << std::endl;
    }
    out.close();
    return true;
}

bool PointsFile::read( std::vector<cv::Point3f> & ats, std::vector<cv::Point3f> & froms )
{
    ats.clear();
    froms.clear();

    in.open( fname );
    bool res = in.is_open();
    if ( !res )
        return false;

    // Extraction of a sub-match
    std::regex base_regex("([\\d.\\e\\-]+)\\.txt");
    std::smatch base_match;
    std::string line;
    while ( std::getline( in, line ) )
    {
        if (std::regex_match(fname, base_match, base_regex))
        {
            // The first sub_match is the whole string; the next
            // sub_match is the first parenthesized expression.
            int sz = base_match.size();
            if ( sz >= 2)
            {
                std::ssub_match base_sub_match = base_match[1];
                std::string base = base_sub_match.str();
                std::cout << base << '\n';
            }
        }
    }

    in.close();
    return true;
}

