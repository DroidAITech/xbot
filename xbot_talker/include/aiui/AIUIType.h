/*
 * AIUIType.h
 *
 *  Created on: 2017年2月17日
 *      Author: hj
 */

#ifndef AIUITYPE_H_
#define AIUITYPE_H_

#include <stdint.h>
#include <sys/types.h>
#include "AIUICommon.h"

namespace aiui {


/**
 * buffer，用于管理一段内存，一般当作带长度的数组用。
 */
class  Buffer
{
public:

    /* flags to use with release() */
    enum {
        eKeepStorage = 0x00000001
    };

    /*! allocate a buffer of size 'size' and acquire() it.
     *  call release() to free it.
     */
    AIUIEXPORT static          Buffer*           alloc(size_t size);

    /*! free the memory associated with the Buffer.
     * Fails if there are any users associated with this Buffer.
     * In other words, the buffer must have been release by all its
     * users.
     */
    AIUIEXPORT static          ssize_t                 dealloc(const Buffer* released);

    //! access the data for read
    AIUIEXPORT inline          const void*             data() const;

    //! access the data for read/write
    AIUIEXPORT inline          void*                   data();

    //! get size of the buffer
    AIUIEXPORT inline          size_t                  size() const;

    //! get back a Buffer object from its data
    AIUIEXPORT static  inline  Buffer*           bufferFromData(void* data);

    //! get back a Buffer object from its data
    AIUIEXPORT static  inline  const Buffer*     bufferFromData(const void* data);

    //! get the size of a Buffer object from its data
    AIUIEXPORT static  inline  size_t                  sizeFromData(const void* data);

    //! edit the buffer (get a writtable, or non-const, version of it)
    AIUIEXPORT                 Buffer*           edit() const;

    //! edit the buffer, resizing if needed
    AIUIEXPORT                 Buffer*           editResize(size_t size) const;

    //! like edit() but fails if a copy is required
    AIUIEXPORT                 Buffer*           attemptEdit() const;

    //! resize and edit the buffer, loose it's content.
    AIUIEXPORT                 Buffer*           reset(size_t size) const;

    //! acquire/release a reference on this buffer
    AIUIEXPORT                 void                    acquire() const;

    /*! release a reference on this buffer, with the option of not
     * freeing the memory associated with it if it was the last reference
     * returns the previous reference count
     */
    AIUIEXPORT                 int32_t                 release(uint32_t flags = 0) const;

    //! returns whether or not we're the only owner
    AIUIEXPORT inline          bool                    onlyOwner() const;

	AIUIEXPORT  Buffer* copy();


private:
        inline Buffer() { }
        inline ~Buffer() { }
        Buffer(const Buffer&);
        Buffer& operator = (const Buffer&);

        // 16 bytes. must be sized to preserve correct alignment.
        mutable int32_t        mRefs;
                size_t         mSize;
                uint32_t       mReserved[2];
};


/**
 * 数据捆绑对象，支持int、string和Buffer*类型数据捆绑传输。
 */

class IDataBundle
{
public:
	AIUIEXPORT virtual ~IDataBundle();

	/**
	 * 创建
	 */
	AIUIEXPORT static IDataBundle* create();

	/**
	 * 销毁
	 */
	virtual void destroy() = 0;

	/**
	 * 从Map中移除key对应的项目
	 */
	virtual bool remove(const char* key) = 0;


	/**
	 * 添加/ 获取 int/string/Buffer 类型数据. 
	 */
	virtual bool putInt(const char* key, int val, bool replace = false) = 0;
	virtual int getInt(const char* key, int defaultVal) = 0;
	virtual bool putString(const char* key, const char* val, bool replace = false) = 0;
	/* getString 返回的数据内存于内部分配，不可在外部delete/free. */
	virtual const char* getString(const char* key, const char* defaultVal) = 0;
	virtual bool putBinary(const char* key, Buffer* binary, bool replace = false) = 0;
	virtual Buffer* getBinary(const char* key) = 0;
};



// ---------------------------------------------------------------------------

const void* Buffer::data() const {
	return this + 1;
}

void* Buffer::data() {
	return this + 1;
}

size_t Buffer::size() const {
	return mSize;
}

Buffer* Buffer::bufferFromData(void* data) {
	return data ? static_cast<Buffer *>(data)-1 : 0;
}

const Buffer* Buffer::bufferFromData(const void* data) {
	return data ? static_cast<const Buffer *>(data)-1 : 0;
}

size_t Buffer::sizeFromData(const void* data) {
	return data ? bufferFromData(data)->mSize : 0;
}

bool Buffer::onlyOwner() const {
	return (mRefs == 1);
}

}

#ifdef AIUI_LIB_COMPILING
#include "aiui_internal/AIUIType_internal.h"
#endif

#endif /* AIUITYPE_H_ */
