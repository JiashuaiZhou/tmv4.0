/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2017, ITU/ISO/IEC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *  * Neither the name of the ITU/ISO/IEC nor the names of its contributors may
 *    be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __PROGRAM_OPTIONS_LITE__
#define __PROGRAM_OPTIONS_LITE__

#include <array>
#include <cassert>
#include <functional>
#include <initializer_list>
#include <iostream>
#include <sstream>
#include <string>
#include <list>
#include <map>
#include <vector>

#define JVET_O0549_ENCODER_ONLY_FILTER_POL \
  1  // JVET-O0549: Encoder-only GOP-based temporal filter. Program Options Lite related changes.

namespace df {
namespace program_options_lite {
  struct Options;

  struct ParseFailure : public std::exception {
    ParseFailure(std::string arg0, std::string val0) throw()
      : arg(arg0), val(val0) {}

    ~ParseFailure() throw(){};

    std::string arg;
    std::string val;

    const char* what() const throw() { return "Option Parse Failure"; }
  };

  struct ErrorReporter {
    ErrorReporter() : is_errored(0) {}
    virtual ~ErrorReporter() {}
    virtual std::ostream& error(const std::string& where = "");
    virtual std::ostream& warn(const std::string& where = "");
    bool                  is_errored;
  };

  extern ErrorReporter default_error_reporter;

  void doHelp(std::ostream& out, Options& opts, unsigned columns = 80);
  void dumpCfg(std::ostream& out, const Options& opts, int indent = 0);
  void
  dumpCfgBySection(std::ostream& out, const Options& opts, int indent = 2);
  void dumpCfg(std::ostream&  out,
               const Options& opts,
               const char*    section,
               int            indent = 0);

  std::list<const char*>
       scanArgv(Options&       opts,
                unsigned       argc,
                const char*    argv[],
                ErrorReporter& error_reporter = default_error_reporter);
  void setDefaults(Options& opts);
  void parseConfigFile(Options&           opts,
                       const std::string& filename,
                       ErrorReporter& error_reporter = default_error_reporter);

  /* Generic parsing */
  template<typename T>
  inline void parse_into(T& dest, const std::string& src) {
    std::istringstream src_ss(src, std::istringstream::in);
    src_ss.exceptions(std::ios::failbit);
    src_ss >> dest;
  }

  /** OptionBase: Virtual base class for storing information relating to a
     * specific option This base class describes common elements.  Type specific
     * information should be stored in a derived class. */
  struct OptionBase {
    OptionBase(const std::string& name, const std::string& desc)
      : opt_string(name), opt_desc(desc){};

    virtual ~OptionBase() {}

    /* parse argument arg, to obtain a value for the option */
    virtual void parse(const std::string& arg, ErrorReporter&) = 0;

    /* set the argument to the default value */
    virtual void setDefault() = 0;

    /* write the default value to out */
    virtual void writeDefault(std::ostream& out) = 0;

    /* write the current value to out */
    virtual void writeValue(std::ostream& out) = 0;

    std::string opt_string;
    std::string opt_desc;
  };

  /** Type specific option storage */
  template<typename T, typename Enable = void>
  struct Option : public OptionBase {
    Option(const std::string& name,
           T&                 storage,
           T                  default_val,
           const std::string& desc)
      : OptionBase(name, desc)
      , opt_storage(storage)
      , opt_default_val(default_val) {}

    void parse(const std::string& arg, ErrorReporter&) {
      try {
        parse_into(opt_storage, arg);
      } catch (...) { throw ParseFailure(opt_string, arg); }
    }

    void setDefault() { opt_storage = opt_default_val; }

    void writeDefault(std::ostream& out) { out << opt_default_val; }

    void writeValue(std::ostream& out) { out << opt_storage; }

    T& opt_storage;
    T  opt_default_val;
  };

  template<typename T>
  struct option_detail_back_inserter {
    static constexpr bool                is_container  = true;
    static constexpr bool                is_fixed_size = false;
    static constexpr std::size_t         size          = 0;
    typedef typename T::value_type       value_type;
    typedef std::back_insert_iterator<T> output_iterator;
    typedef typename T::const_iterator   forward_iterator;

    template<typename T2>
    static void assign(T& container, T2&& init) {
      container.assign(init);
    }
    static void            clear(T& container) { container.clear(); }
    static bool            valid(const T&, output_iterator) { return true; }
    static output_iterator make_output_iterator(T& container) {
      return std::back_inserter(container);
    }
  };

  template<class Container>
  struct option_detail;

  template<class T>
  struct option_detail {
    static constexpr bool is_container = false;
  };

  template<typename... Ts>
  struct option_detail<std::vector<Ts...>>
    : public option_detail_back_inserter<std::vector<Ts...>> {};

  template<typename... Ts>
  struct option_detail<std::list<Ts...>>
    : public option_detail_back_inserter<std::list<Ts...>> {};

  template<typename T1, std::size_t N>
  struct option_detail<std::array<T1, N>> {
    static constexpr bool                              is_container  = true;
    static constexpr bool                              is_fixed_size = true;
    static constexpr std::size_t                       size          = N;
    typedef T1                                         value_type;
    typedef typename std::array<T1, N>                 T;
    typedef typename std::array<T1, N>::iterator       output_iterator;
    typedef typename std::array<T1, N>::const_iterator forward_iterator;

    template<typename T2>
    static void assign(T& container, T2&& init) {
      assert(init.size() <= container.size());
      std::copy(init.begin(), init.end(), container.begin());
    }

    static void clear(T& container) {
      std::fill_n(std::begin(container), N, value_type());
    }

    static output_iterator make_output_iterator(T& container) {
      return container.begin();
    }
  };

  template<typename T, std::size_t N>
  struct option_detail<T[N]> {
    static constexpr bool        is_container  = true;
    static constexpr bool        is_fixed_size = true;
    static constexpr std::size_t size          = N;
    typedef T                    value_type;
    typedef T*                   output_iterator;
    typedef const T*             forward_iterator;

    static void assign(T* container, std::initializer_list<T> init) {
      assert(init.size() <= N);
      std::copy(init.begin(), init.end(), container);
    }

    static void clear(T* container) {
      std::fill_n(container, N, value_type());
    }

    static output_iterator make_output_iterator(T* container) {
      return container;
    }
  };

  /**
     * Container type specific option storage.
     *
     * The option's argument is split by ',' and whitespace.  Runs of
     * whitespace are ignored. Compare:
     *  "a, b,c,,e" = {T1(a), T1(b), T1(c), T1(), T1(e)}, vs.
     *  "a  b c  e" = {T1(a), T1(b), T1(c), T1(e)}.
     *
     * NB: each application of this option overwrites the previous instance,
     *     in exactly the same way that normal (non-container) options to.
     */
  template<typename T>
  struct Option<T,
                typename std::enable_if<option_detail<T>::is_container>::type>
    : public OptionBase {
    typedef option_detail<T>            detail;
    typedef typename detail::value_type elmt_type;

    Option(const std::string&               name,
           T&                               storage,
           std::initializer_list<elmt_type> default_val,
           const std::string&               desc)
      : OptionBase(name, desc)
      , opt_storage(storage)
      , opt_default_val(default_val) {}

    void parse(const std::string& arg, ErrorReporter&) {
      /* ensure that parsing overwrites any previous value */
      detail::clear(opt_storage);
      auto it    = detail::make_output_iterator(opt_storage);
      auto count = detail::size;

      /* effectively map parse . split m/, /, @arg */
      std::string::size_type pos = 0;
      do {
        if (detail::is_fixed_size && !count--)
          throw ParseFailure(opt_string, "too many arguments");

        /* skip over preceeding spaces */
        pos             = arg.find_first_not_of(" \t", pos);
        auto        end = arg.find_first_of(", \t", pos);
        std::string sub_arg(arg, pos, end - pos);

        try {
          elmt_type value;
          parse_into(value, sub_arg);
          *it++ = value;
        } catch (...) { throw ParseFailure(opt_string, sub_arg); }

        pos = end + 1;
      } while (pos != std::string::npos + 1);
    }

    void setDefault() { detail::assign(opt_storage, opt_default_val); }

    void writeDefault(std::ostream& out) {
      out << '"';
      bool first = true;
      for (const auto& val : opt_default_val) {
        if (!first) out << ',';
        out << val;
        first = false;
      }
      out << '"';
    }

    void writeValue(std::ostream& out) {
      bool first = true;
      for (const auto& val : opt_storage) {
        if (!first) out << ", ";
        out << val;
        first = false;
      }
    }

    T&                               opt_storage;
    std::initializer_list<elmt_type> opt_default_val;
  };

  /* string parsing is specialized -- copy the whole string, not just the
     * first word */
  template<>
  inline void Option<std::string>::parse(const std::string& arg,
                                         ErrorReporter&) {
    opt_storage = arg;
  }

  /* strings are pecialized -- output whole string rather than treating as
     * a sequence of characters */
  template<>
  inline void Option<std::string>::writeDefault(std::ostream& out) {
    out << '"' << opt_default_val << '"';
  }

  /* strings are pecialized -- output whole string rather than treating as
     * a sequence of characters */
  template<>
  inline void Option<std::string>::writeValue(std::ostream& out) {
    out << '"' << opt_storage << '"';
  }

  /** Option class for argument handling using a user provided function */
  struct OptionFunc : public OptionBase {
    typedef void(Func)(Options&, const std::string&, ErrorReporter&);

    OptionFunc(const std::string&  name,
               Options&            parent_,
               std::function<Func> func_,
               const std::string&  desc)
      : OptionBase(name, desc), parent(parent_), func(func_) {}

    void parse(const std::string& arg, ErrorReporter& error_reporter) {
      func(parent, arg, error_reporter);
    }

    void setDefault() { return; }

    void writeDefault(std::ostream& out) {
      /* there is no default */
      out << "...";
    }

    void writeValue(std::ostream& out) {
      /* there is no vaule */
      out << "...";
    }

  private:
    Options&            parent;
    std::function<Func> func;
  };

  struct Section {
    Section(const std::string& name_) : name(name_) {}

    std::string name;
  };

  class OptionSpecific;
  struct Options {
    ~Options();

    OptionSpecific addOptions();

    struct Names {
      Names() : opt(0){};
      ~Names() {
        if (opt) { delete opt; }
      }
      std::list<std::string> opt_long;
      std::list<std::string> opt_short;
#if JVET_O0549_ENCODER_ONLY_FILTER_POL
      std::list<std::string> opt_prefix;    
#endif
      OptionBase*            opt;
    };

    void addOption(OptionBase* opt);

    typedef std::list<Names*> NamesPtrList;
    NamesPtrList              opt_list;

    // beginning of each option section
    typedef std::pair<Section, NamesPtrList::const_iterator> SectionPtr;
    std::list<SectionPtr>                                    sections;

    typedef std::map<std::string, NamesPtrList> NamesMap;
    NamesMap                                    opt_long_map;
    NamesMap                                    opt_short_map;
#if JVET_O0549_ENCODER_ONLY_FILTER_POL
    NamesMap                                    opt_prefix_map;
#endif                                                         
  };

  /* Class with templated overloaded operator(), for use by Options::addOptions() */
  class OptionSpecific {
  public:
    OptionSpecific(Options& parent_) : parent(parent_) {}

    /**
       * Add option described by name to the parent Options list,
       *   with storage for the option's value
       *   with default_val as the default value
       *   with desc as an optional help description
       */
    template<typename T>
    OptionSpecific& operator()(const std::string& name,
                               T&                 storage,
                               T                  default_val,
                               const std::string& desc = "") {
      parent.addOption(new Option<T>(name, storage, default_val, desc));
      return *this;
    }

    /**
       * Add option described by name to the parent Options list,
       *   with storage for the option's value
       *   with default_val as the default value
       *   with desc as an optional help description
       */
    template<typename T>
    OptionSpecific& operator()(
      const std::string&                                           name,
      T&                                                           storage,
      std::initializer_list<typename option_detail<T>::value_type> defval,
      const std::string&                                           desc = "") {
      parent.addOption(new Option<T>(name, storage, defval, desc));
      return *this;
    }

    template<typename T>
    OptionSpecific& operator()(const std::string& name,
                               T*                 storage,
                               T                  default_val,
                               unsigned           uiMaxNum,
                               const std::string& desc = "") {
      std::string cNameBuffer;
      std::string cDescriptionBuffer;

      for (unsigned int uiK = 0; uiK < uiMaxNum; uiK++) {
        // it needs to be reset when extra digit is added, e.g. number 10 and above
        cNameBuffer.resize(name.size() + 10);
        cDescriptionBuffer.resize(desc.size() + 10);

        // isn't there are sprintf function for string??
        sprintf((char*)cNameBuffer.c_str(), name.c_str(), uiK, uiK);
        sprintf((char*)cDescriptionBuffer.c_str(), desc.c_str(), uiK, uiK);

        size_t pos = cNameBuffer.find_first_of('\0');
        if (pos != std::string::npos) { cNameBuffer.resize(pos); }

        parent.addOption(new Option<T>(
          cNameBuffer, (storage[uiK]), default_val, cDescriptionBuffer));
      }

      return *this;
    }

    template<typename T>
    OptionSpecific& operator()(const std::string& name,
                               T**                storage,
                               T                  default_val,
                               unsigned           uiMaxNum,
                               const std::string& desc = "") {
      std::string cNameBuffer;
      std::string cDescriptionBuffer;

      for (unsigned int uiK = 0; uiK < uiMaxNum; uiK++) {
        // it needs to be reset when extra digit is added, e.g. number 10 and above
        cNameBuffer.resize(name.size() + 10);
        cDescriptionBuffer.resize(desc.size() + 10);

        // isn't there are sprintf function for string??
        sprintf((char*)cNameBuffer.c_str(), name.c_str(), uiK, uiK);
        sprintf((char*)cDescriptionBuffer.c_str(), desc.c_str(), uiK, uiK);

        size_t pos = cNameBuffer.find_first_of('\0');
        if (pos != std::string::npos) cNameBuffer.resize(pos);

        parent.addOption(new Option<T>(
          cNameBuffer, *(storage[uiK]), default_val, cDescriptionBuffer));
      }

      return *this;
    }
    /**
       * Add option described by name to the parent Options list,
       *   with desc as an optional help description
       * instead of storing the value somewhere, a function of type
       * OptionFunc::Func is called.  It is upto this function to correctly
       * handle evaluating the option's value.
       */
    OptionSpecific& operator()(const std::string&              name,
                               std::function<OptionFunc::Func> func,
                               const std::string&              desc = "") {
      parent.addOption(new OptionFunc(name, parent, func, desc));
      return *this;
    }

    /**
       * Add a section header to the options list.
       */
    OptionSpecific& operator()(const Section& section) {
      parent.sections.emplace_back(section, parent.opt_list.cend());
      return *this;
    }

  private:
    Options& parent;
  };

}  // namespace program_options_lite
}  // namespace df

#endif
