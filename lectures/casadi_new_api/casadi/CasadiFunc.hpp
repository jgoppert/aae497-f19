#include "casadi/mem.h"


class CasadiFunc
{
private:
	casadi_mem *_mem = nullptr;
public:
	CasadiFunc(casadi_functions *f) :
		_mem(casadi_alloc(f))
	{
	}
	~CasadiFunc()
	{
		casadi_free(_mem);
	}
	void res(int i, casadi_real *v)
	{
		_mem->res[i] = v;
	}
	void arg(int i, const casadi_real *v)
	{
		_mem->arg[i] = v;
	}
	void eval()
	{
		casadi_eval(_mem);
	}
	casadi_io in(int i)
	{
		return _mem->in[i];
	}
	casadi_io out(int i)
	{
		return _mem->out[i];
	}
};
