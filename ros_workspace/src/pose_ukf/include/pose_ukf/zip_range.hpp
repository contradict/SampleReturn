#include <boost/iterator/zip_iterator.hpp>
#include <boost/range/iterator_range.hpp>
#pragma once
template<class... Conts>
auto zip_range(Conts&... conts)
      -> decltype(boost::make_iterator_range(
                    boost::make_zip_iterator(boost::make_tuple(conts.begin()...)),
                      boost::make_zip_iterator(boost::make_tuple(conts.end()...))))
{
      return {boost::make_zip_iterator(boost::make_tuple(conts.begin()...)),
                    boost::make_zip_iterator(boost::make_tuple(conts.end()...))};
}
