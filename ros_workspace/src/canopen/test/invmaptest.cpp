#include <iostream>
#include <map>
#include "invertmap.h"

int main(int argc, char **argv)
{
    std::map< std::string, int> m1;
    std::map< char, int> m2;
    std::map< int, std::string> m1_inv;
    std::map< int, char> m2_inv;

    m1[std::string("a")]=1;
    m1[std::string("b")]=2;
    m1[std::string("c")]=3;
    m1[std::string("d")]=4;
    m1[std::string("e")]=5;
    m2['a']=1;
    m2['b']=2;
    m2['c']=3;
    m2['d']=4;
    m2['e']=5;

    m1_inv = InvertMap(m1);
    m2_inv = InvertMap(m2);

    std::map< int, std::string >::iterator inv_map1it;
    for(inv_map1it=m1_inv.begin(); inv_map1it != m1_inv.end(); inv_map1it++) {
        std::cout << (*inv_map1it).first << " => " << (*inv_map1it).second << std::endl;
    }
    std::map< int, char >::iterator inv_map2it;
    for(inv_map2it=m2_inv.begin(); inv_map2it != m2_inv.end(); inv_map2it++) {
        std::cout << (*inv_map2it).first << " => " << (*inv_map2it).second << std::endl;
    }
    return 0;
} 
