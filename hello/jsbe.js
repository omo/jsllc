
__JSBE = {

 Ptr: function(obj) {
    this.__ref = obj;
 },

 MemoryArray: function(arr, off) {
    this.arr = arr || [];
    this.off = off || 0;
    this.element_at = function() {
      var sum = 0;
      for (var i=0; i<arguments.length; i++) { sum += arguments[i]; }
      return new __JSBE.MemoryArray(this.arr, sum);
    };
    this.ref = function() {
      var toret = this.arr[this.off];
      return undefined == toret ? 0 : toret;
    };
    this.set_ref = function(ch) {
      this.arr[this.off] = ch;
    }
 },

 cast: function(obj) {
    return obj;
 },

 make_memory: function() {
    return new __JSBE.MemoryArray();
 },

 make_ptr: function(obj) {
    return new __JSBE.Ptr(obj);
 },

 nPrinted: 0,

 print: function(val) {
    if (10 < __JSBE.nPrinted++) { throw "Detect Loop!"; }
    console.log(val);
 },

 decodeCharCode: function (code) {
    var u = (code%256).toString(16);
    if (1 == u.length)  { u = '0' + u; }
    return decodeURIComponent('%'+u);
 }

};

String.prototype.toIntArray = function() {
  return this.toArray().map(function(ch) {
      return ch.charCodeAt(0);
  });
};

/*
 * instrinsics
 */
// http://llvm.org/docs/LangRef.html#int_memcpy
function llvm_memcpy_i32(dst, src, size, align) { // align is ignored...
  for (var i=0; i<size; i++) {
    dst.arr[i] = src.arr[i];
  }
}

/*
 * externals
 */
function putc(ch) {
  __JSBE.print(__JSBE.decodeCharCode(ch));
}

function extern_puts(arg) {
  __JSBE.print(arg.arr.inject("", function(a,i) {
      if (0 < i) {
        a += __JSBE.decodeCharCode(i);
      }
      return a;
  }));
}
