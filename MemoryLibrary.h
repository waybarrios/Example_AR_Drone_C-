/* 
Feel free to modify the code to your needs
 */
#ifndef __BUFFER_HEADER__
#define __BUFFER_HEADER__

#include "dll.ix.h"

#ifdef OSX_MACH_O   // bsd does not have malloc.h, everything in malloc.h is in stdlib.h
#define ON_FREEBSD
#endif

#ifdef __APPLE__   // bsd does not have malloc.h, everything in malloc.h is in stdlib.h
#define ON_FREEBSD
#endif

#ifndef ON_FREEBSD 
#include <malloc.h>
#else
#include <stdlib.h>
#endif


#include <memory.h>
#include "dll.ix.h"

namespace MemoryLibrary
{
  template <typename T>
  void Copy(T *destination, T *source, int iCnt)
  {
    ::memcpy((void*)destination, (void*)source, iCnt * sizeof(T));
  }

  template <typename T>
  void Zero(T *destination, int iCnt)
  {
    ::memset((void*)destination, 0, iCnt * sizeof(T));
  }

  template <typename T>
  void Set(T *destination, unsigned char v, int iCnt)
  {
    ::memset((void*)destination, v, iCnt * sizeof(T));
  }

  class Buffer
  {
  protected:
    void* m_pBuffer;
    unsigned long m_uiSize;
  public:
    Buffer(void)
    {
      m_pBuffer = NULL;
      m_uiSize = 0;
    }

    Buffer(void* p, unsigned long s)
    {
      Set(p, s);
    }

    virtual ~Buffer(void)
    {
      m_pBuffer = NULL;
      m_uiSize = 0;
    }

    bool IsValid(void)
    {
      return m_pBuffer != NULL ? true : false;
    }

    bool IsEqualTo(MemoryLibrary::Buffer& p)
    {
      return this->Size() == p.Size() && ::memcmp(this->Ptr(), p.Ptr(), Size()) == 0;
    }

    void* Ptr(void)
    {
      return m_pBuffer;
    }
    
    void* PointerToThisBuffer()
    {
      return Ptr();
    }
    
    unsigned char* memoryLocationAsByte()
    {
      return (unsigned char*)Ptr();
    }
    
    unsigned long size(void)
    {
      return m_uiSize;
    }

    unsigned long Size(void)
    {
      return m_uiSize;
    }

    void Set(void* p, unsigned long s)
    {
      m_pBuffer = p;
      m_uiSize = s;
    }

    void Zero(void)
    {
      ::memset(m_pBuffer, 0, m_uiSize);
    }
    
    void CopyFrom(void* p)
    {
      ::memcpy(m_pBuffer, p, m_uiSize);
    }

    void CopyFrom(void* p, unsigned int s)
    {
      ::memcpy(m_pBuffer, p, s);
    }

    void CopyFrom(MemoryLibrary::Buffer& b)
    {
      CopyFrom(b.Ptr(), b.Size());
    }

    void CopyTo(void* p, unsigned long s)
    {
      ::memcpy(p, m_pBuffer, s);
    }

    void CopyTo(MemoryLibrary::Buffer& b)
    {
      CopyTo(b.Ptr(), b.Size());
    }
    
    template <typename T>
    void CopyFrom(T* p)
    {
      if(sizeof(T) == Size())
        CopyFrom((void*)p, sizeof(T));
      else 
      {
        // need to pump error msg here
      }
    }
    
    template <typename T>
    void CopyTo(T* p)
    {
      if(Size() == sizeof(T))
        CopyTo((void*)p, sizeof(T));
      else 
      {
        // need to pump error
      }

    }
    
    template <typename T>
    void CopyFrom(T* p, unsigned long nBlocks)
    {
      if(nBlocks * sizeof(T) == Size())
        CopyFrom((void*)p, nBlocks * sizeof(T));
      else 
      {
        // need to pump error msg here
      }
    }
    
    template <typename T>
    void CopyTo(T* p, unsigned long nBlocks)
    {
      if(Size() == nBlocks * sizeof(T))
        CopyTo((void*)p, nBlocks * sizeof(T));
      else 
      {
        // need to pump error
      }
    }
    
    template <typename T>
    T makeValueFromOffset(int offset)
    {
      T result = (T)0;
      if((offset + sizeof(T)) < Size())
      {
        unsigned char *pStart = ((unsigned char*)m_pBuffer) + (unsigned char)offset;
        ::memcpy(&result, pStart, sizeof(T));
      }
      
      return result;
    }
    
    template <typename T>
    T MakeValueFromOffset(int offset)
    {
      T result = (T)0;
      if((offset + sizeof(T)) < Size())
      {
        unsigned char *pStart = ((unsigned char*)m_pBuffer) + (unsigned char)offset;
        ::memcpy(&result, pStart, sizeof(T));
      }
      
      return result;
    }
    
    unsigned char& byteValueAt(int i)
    {
      unsigned char* byte = (unsigned char*)m_pBuffer;
      return byte[i]; // no range check;
    }
    
    unsigned char& operator[](int i)
    {
      return byteValueAt(i);
    }
  };

  ///////////////////////////////////////////////////
  template<int BufferSizeInByte>
  class StaticBuffer :public Buffer
  {
    unsigned char myBuffer[BufferSizeInByte];
    
  public:
    StaticBuffer() :Buffer((void*)myBuffer, BufferSizeInByte)
    {
    }
  };
  
  //////////////////////////////
  template <typename T>
  class DataTypedBuffer :public Buffer
  {
    T myType;
  public:
    DataTypedBuffer() :Buffer((void*)myType, sizeof(T))
    {
    }
  };
  
  ////////////////////////////////////////
  class DynamicBuffer :public Buffer
  {
  public:
    DynamicBuffer() :Buffer(NULL, 0)
    {
    }
    
    virtual ~DynamicBuffer(void)
    {
      Free();
    }
   
    virtual bool Allocate(unsigned long nByte)
    {      
      // new size?
      if(nByte != m_uiSize)
      {
        Free();
        m_pBuffer = ::malloc(nByte);
        if(NULL == m_pBuffer)
          return false;
        else
        {
          m_uiSize = nByte;
          return true;
        }
      }
      else
      {
        // buffer is already been allocated and 
        // same size with nByte, so just return true;
        return true;
      }
    }
    
    virtual void Free(void)
    {
      if(NULL != m_pBuffer)
      {
        ::free(m_pBuffer);
        m_pBuffer = NULL;
        m_uiSize = 0;
      }
    }
    
    bool ReadFromFile(const char* szFilename)
    {
      FILE *fp;
      long len;
      fp = ::fopen(szFilename,"rb");
      if(NULL == fp)
        return false;
      ::fseek(fp,0,SEEK_END); 
      len=ftell(fp); 
      fseek(fp,0,SEEK_SET); 
      Allocate(len);
      ::fread(memoryLocationAsByte(), len, 1, fp);
      ::fclose(fp);
      
      return true;
    }
  };

  ///////////////////////////////////////////////////////////////////
  template <typename T> 
  class IndexableBuffer // rename to IndexableBuffer
  {
    bool m_bFreeBuffer;
    MemoryLibrary::DynamicBuffer m_buffer;
    T* m_ptr;
    unsigned int m_nBlock;
  public:
    IndexableBuffer(void)
    {
      m_bFreeBuffer = true;
      m_nBlock = 0;
      m_ptr = NULL;
    }

    IndexableBuffer(int iSize)
    {
      m_bFreeBuffer = true;
      m_nBlock = 0;
      m_ptr = NULL;
      Allocate(iSize);
    }

    IndexableBuffer(T* ptrExtBuffer, int iSize)
    {
      m_bFreeBuffer = true;
      m_nBlock = 0;
      m_ptr = NULL;
      SetExternalBuffer(ptrExtBuffer, iSize);
    }

    virtual ~IndexableBuffer(void)
    {
      Free();
    }

    void SetExternalBuffer(T* ptr, unsigned int size)
    {
      Free();
      m_bFreeBuffer = false;
      m_ptr = ptr;
      m_buffer.Set((void*)ptr, size);
      m_nBlock = size;
    }

    void Copy(MemoryLibrary::IndexableBuffer<T>& a)
    {
      Copy(a.RawBuffer(), a.Size());
    }

    void Copy(T* Ptr, unsigned int iSize)
    {
      Allocate(iSize);
      m_buffer.CopyFrom((void*)Ptr, iSize * sizeof(T));
    }

    void Fill(T a)
    {
      for(unsigned int i = 0; i < this->Size(); i++)
        At(i) = a;
    }

    virtual bool Allocate(unsigned int nBlock, bool bZeroOutBuffer=true)
    {
      m_bFreeBuffer = true;
      m_nBlock = nBlock;
      if(true == m_buffer.Allocate(sizeof(T) * nBlock))
      {
        if(true == bZeroOutBuffer)
          m_buffer.Zero();
        m_ptr = (T*)m_buffer.Ptr();
        return true;
      }
      else
      {
        m_ptr = NULL;
        return false;
      }
    }

    virtual void Free(void)
    {
      if(true == m_bFreeBuffer)
      {
        m_nBlock = 0;
        m_buffer.Free();
        m_ptr = NULL;
      }
      else // this mean I am operating on external buffer, see SetExternalBuffer
      {
        m_buffer.Set(NULL, 0); // so DynamicBuffer won't delete the external ptr
        m_nBlock = 0;
      }
    }

    unsigned int Size(void)
    {
      return m_nBlock;
    }

    unsigned int RawSize(void)
    {
      return m_buffer.Size();
    }

    T* RawBuffer(void)
    {
      return m_ptr; 
    }

    T* Memory(void)
    {
      return m_ptr;
    }

    T& At(unsigned int iIdx)
    {
      //Debug::CheckUpperBoundRange(Size() - 1, iIdx, "MemoryLibrary::IndexableBuffer::At");
      return m_ptr[iIdx];
    }

    T &operator[](int i)
    {
      return At(i);
    }
  };
  

} // namespace MemoryLibrary

#endif

