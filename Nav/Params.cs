using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Nav
{
    public class Params
    {
        public Params(string[] args)
        {
            if (args == null)
                return;

            string last_param_name = "";

            foreach (string arg in args)
            {
                if (arg.StartsWith("-"))
                {
                    if (last_param_name != "")
                        m_Params[last_param_name] = "__present";

                    last_param_name = arg.TrimStart('-');
                }
                else
                {
                    if (last_param_name != "")
                    {
                        m_Params[last_param_name] = arg;
                        last_param_name = "";
                    }
                    else
                        throw new Exception("Invalid parameter: " + arg);
                }
            }

            if (last_param_name != "")
                m_Params[last_param_name] = "__present";
        }

        public bool GetParam(string name, out string value)
        {
            return m_Params.TryGetValue(name, out value);
        }

        public bool HasParam(string name)
        {
            return m_Params.ContainsKey(name);
        }

        private Dictionary<string, string> m_Params = new Dictionary<string, string>();
    }
}
