/*
 * Copyright (c) 2008, Maxim Likhachev
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Carnegie Mellon University nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _HEAP_H_
#define _HEAP_H_

#include <stdexcept>
#include <string>

// the maximum size of the heap
#define HEAPSIZE 20000000
#define HEAPSIZE_INIT 5000
#define INFINITECOST 1000000000

class SBPL_Exception : public std::runtime_error
{
public:
  explicit SBPL_Exception(const std::string& what_arg = "SBPL has encountered a fatal error!")
    : std::runtime_error(what_arg)
  {
    printf("%s\n", what_arg.c_str());
  }

  explicit SBPL_Exception(const char* what_arg) : std::runtime_error(what_arg)
  {
    printf("%s\n", what_arg);
  }

  virtual ~SBPL_Exception() throw()
  {
  }

private:
};

class AbstractSearchState
{
public:
  int heapindex;

public:
  AbstractSearchState()
  {
    heapindex = 0;
  }
  virtual ~AbstractSearchState()
  {
  }
};

struct HEAPINTELEMENT
{
  AbstractSearchState* heapstate; 
  int key; 
};

typedef struct HEAPINTELEMENT heapintelement; 


class CIntHeap
{
public:
  int percolates;  
  heapintelement* heap; 
  int currentsize;
  int allocated; 
public:
  CIntHeap();
  CIntHeap(int initial_size);
  ~CIntHeap();
public:
  bool emptyheap();
  bool fullheap();
  bool inheap(AbstractSearchState* AbstractSearchState);
  int getkeyheap(AbstractSearchState* AbstractSearchState);
  void makeemptyheap();
  void insertheap(AbstractSearchState* AbstractSearchState, int key);
  void deleteheap(AbstractSearchState* AbstractSearchState);
  void updateheap(AbstractSearchState* AbstractSearchState, int NewKey);
  AbstractSearchState* getminheap();
  AbstractSearchState* getminheap(int& ReturnKey);
  int getminkeyheap();
  AbstractSearchState* deleteminheap();
  void makeheap();

private:
  void percolatedown(int hole, heapintelement tmp); 
  void percolateup(int hole, heapintelement tmp); 
  void percolateupordown(int hole, heapintelement tmp);

  void growheap();
  void sizecheck();
};

#endif  // _HEAP_H_