/*****************************************************************************
 * PROJECT: Gamebot
 *
 * (c) Copyright 2010 Maxim Makatchev All rights reserved.
 *
 * FILE: exparser.h
 *
 * ABSTRACT: Math and string expression parser
 *
 *
 *
 ****************************************************************/

#ifndef EXPARSER_H 
#define EXPARSER_H

enum Associativity {ASSOC_NONE=0, ASSOC_LEFT, ASSOC_RIGHT};
enum TermType {UNKNOWN=0, NUMBER, OPERATOR, FUNCTION, VARIABLE, LEFT_PAREN,
               RIGHT_PAREN, COMMA, STRINGTERM};

void initExpressionParsing();

class UnitRecord{
  public:
        // default constructor
    UnitRecord(): factor(1.0) {}
  public:
    UnitRecord(string aUnit, double f, string sUnit): unit(aUnit),
        factor(f), standard_unit(sUnit) {}
        
  public:
    string unit;
    double factor;
    string standard_unit;
};


class ExpressionParser{

  public:
        /* default constructor is here because of the reference members that must be
         * initialized in the initializer list of class constructor */
    ExpressionParser(Conjunction &lBindings):
        bindings(lBindings) {
            /* units */
        UnitRecord u1("ms", 0.001, "s");
        this->measures.push_back(u1);
        UnitRecord u2("min", 60, "s");
        this->measures.push_back(u2);
        UnitRecord u3("h", 3600, "s");
        this->measures.push_back(u3);
        UnitRecord u4("d", 86400, "s");
        this->measures.push_back(u4);
        UnitRecord u5("s", 1, "s");
        this->measures.push_back(u5);

    }

  public:
    string getToken(string anExp, size_t prefix_ind, size_t &end_ind);
    string getValue(string varname);

    
    string eval(string aType, string anExp, string &units);
    string substitute(string anExp);
    string evalStringExpression(string anExp);

    bool convertIntoStandardUnits2(string &num_str, string &units_orig);
    
    bool is_operator(const string op);
    bool is_string_operator(const string op);
    bool is_function(const string op);
    bool is_string_function(const string op);
    bool is_variable(const string op);
    bool evalNumberExpressionShuntingYard(string anExp, string &unit, string &res);
    bool evalStringExpressionShuntingYard(string anExp, string &unit, string &res);
    bool readTerm(const string &input, unsigned int &strpos,
                  string &term, TermType &term_type);
    bool readStringTerm(const string &input, unsigned int &strpos,
                  string &term, TermType &term_type);
    bool shuntingYard(const string &input, deque<pair<string, TermType> > &output_q);
    bool shuntingYardString(const string &input, deque<pair<string, TermType> > &output_q);
    bool evalFunctionOrOperator(pair<string, TermType> &op_type_pair,
                                deque<pair<string, TermType> > &arg_dq,
                                string &res);
    bool evalStringFunctionOrOperator(pair<string, TermType> &op_type_pair,
                                deque<pair<string, TermType> > &arg_dq,
                                string &res);
  public:
        //Conjunction& gBindings;
    Conjunction& bindings;
    list<UnitRecord> measures;
};


class Operator {

  public:
        // default constructor
    Operator() {}
  public:
    Operator(string op, int prec, Associativity assoc, bool unary):
        op(op), prec(prec), assoc(assoc), unary(unary) {}

  public:
    string op;
    int prec;
    Associativity assoc;
    bool unary;

};


class Function {

  public:
        // default constructor
    Function() {}
 public:
    Function(string fun, int arg_count):
        fun(fun), arg_count(arg_count) {}

  public:
    string fun;
    int arg_count;

};

inline bool isREFunction(const string &input) {
    if ((input=="FullMatch") || (input == "PartialMatch")) {
        return true;
    }
    return false;
}


#endif // EXPARSER_H
