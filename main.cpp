#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <unordered_map>
#include <memory>
#include <cctype>
#include <stack>
#include <cstdlib>
using namespace std;

// ================== Token 类型 ==================
enum class TokenType {
    Identifier, Number, String, Print, Eq,
    Assign, Plus, Minus, Multiply, Divide,
    LParen, RParen, LBrace, RBrace,
    Semicolon, Eof ,If, While, Else, Neq, Lt, Gt, Lte, Gte,
};

struct Token {
    TokenType type;
    string value;
};

// ================== 词法分析器 (Lexer) ==================
class Lexer {
public:
    explicit Lexer(const string& input) : input(input), pos(0) {}
    Token nextToken() {
        skipWhitespace();
        if (pos >= input.size()) return {TokenType::Eof, ""};
        char c = input[pos];
        if (isalpha(c)) {
            string ident;
            while (pos < input.size() && isalnum(input[pos])) {
                ident += input[pos++];
            }
            if (ident == "print") return {TokenType::Print, ident};
            if (ident == "if") return {TokenType::If, ident};
            if (ident == "while") return {TokenType::While, ident};
            if (ident == "else") return {TokenType::Else, ident};
            return {TokenType::Identifier, ident};
        }
        if (isdigit(c)) {
            string num;
            while (pos < input.size() && isdigit(input[pos])) {
                num += input[pos++];
            }
            return {TokenType::Number, num};
        }
        if (c == '"') {
            string str;
            pos++; // skip "
            while (pos < input.size() && input[pos] != '"') {
                if (input[pos] == '\\' && pos + 1 < input.size()) {
                    pos++;
                    switch (input[pos]) {
                        case 'n': str += '\n'; break;
                        case 't': str += '\t'; break;
                        case '\\': str += '\\'; break;
                        case '"': str += '"'; break;
                        default: str += input[pos]; break;
                    }
                } else {
                    str += input[pos];
                }
                pos++;
            }
            if (pos < input.size() && input[pos] == '"') pos++; // skip closing "
            return {TokenType::String, str};
        }
        switch (c) {
            case '=': pos++; return (pos < input.size() && input[pos] == '=') ?
                                    Token{TokenType::Eq, "=="} : Token{TokenType::Assign, "="};
            case '+': pos++; return {TokenType::Plus, "+"};
            case '-': pos++; return {TokenType::Minus, "-"};
            case '*': pos++; return {TokenType::Multiply, "*"};
            case '/': pos++; return {TokenType::Divide, "/"};
            case '(': pos++; return {TokenType::LParen, "("};
            case ')': pos++; return {TokenType::RParen, ")"};
            case '{': pos++; return {TokenType::LBrace, "{"};
            case '}': pos++; return {TokenType::RBrace, "}"};
            case ';': pos++; return {TokenType::Semicolon, ";"};

            case '!': pos++; if (pos < input.size() && input[pos] == '=') { pos++; return {TokenType::Neq, "!="}; } break;
            case '<': pos++; if (pos < input.size() && input[pos] == '=') { pos++; return {TokenType::Lte, "<="}; } return {TokenType::Lt, "<"};
            case '>': pos++; if (pos < input.size() && input[pos] == '=') { pos++; return {TokenType::Gte, ">="}; } return {TokenType::Gt, ">"};
            default:
                throw runtime_error("Unknown character: " + string(1, c));
        }
        return {TokenType::Eof, ""};
    }

private:
    string input;
    size_t pos;
    void skipWhitespace() {
        while (pos < input.size() && isspace(input[pos])) pos++;
    }
};

// ================== AST节点 ==================
struct AstNode {
    virtual ~AstNode() = default;
    virtual void generateIR(vector<string>& ir, unordered_map<string, int>& vars, unordered_map<string, string>& strings) const = 0;
    virtual bool isString() const { return false; }
};
struct IfNode : AstNode {
    unique_ptr<AstNode> condition;
    vector<unique_ptr<AstNode>> thenBody;
    vector<unique_ptr<AstNode>> elseBody;

    IfNode(unique_ptr<AstNode> cond, vector<unique_ptr<AstNode>> thn, vector<unique_ptr<AstNode>> els)
            : condition(move(cond)), thenBody(move(thn)), elseBody(move(els)) {}

    void generateIR(vector<string>& ir, unordered_map<string, int>& vars, unordered_map<string, string>& strings) const override {
        static int labelId = 0;
        int id = labelId++;

        condition->generateIR(ir, vars, strings);
        ir.push_back("JZ else_label_" + to_string(id));

        for (const auto& stmt : thenBody) {
            stmt->generateIR(ir, vars, strings);
        }

        ir.push_back("JMP end_if_label_" + to_string(id));
        ir.push_back("LABEL else_label_" + to_string(id));

        for (const auto& stmt : elseBody) {
            stmt->generateIR(ir, vars, strings);
        }

        ir.push_back("LABEL end_if_label_" + to_string(id));
    }
};
struct WhileNode : AstNode {
    unique_ptr<AstNode> condition;
    vector<unique_ptr<AstNode>> body;

    WhileNode(unique_ptr<AstNode> cond, vector<unique_ptr<AstNode>> bd)
            : condition(move(cond)), body(move(bd)) {}

    void generateIR(vector<string>& ir, unordered_map<string, int>& vars, unordered_map<string, string>& strings) const override {
        static int labelId = 0;
        int start = labelId++;
        int end = labelId++;

        ir.push_back("LABEL loop_condition_label_" + to_string(start));
        condition->generateIR(ir, vars, strings);
        ir.push_back("JZ loop_end_label_" + to_string(start));

        for (const auto& stmt : body) {
            stmt->generateIR(ir, vars, strings);
        }

        ir.push_back("JMP loop_condition_label_" + to_string(start));
        ir.push_back("LABEL loop_end_label_" + to_string(start));
    }
};
struct NumberNode : AstNode {
    string value;
    explicit NumberNode(string v) : value(v) {}
    void generateIR(vector<string>& ir, unordered_map<string, int>&, unordered_map<string, string>&) const override {
        ir.push_back("PUSH " + value);
    }
};

struct StringNode : AstNode {
    string label;
    explicit StringNode(string l) : label(l) {}
    bool isString() const override { return true; }
    void generateIR(vector<string>& ir, unordered_map<string, int>&, unordered_map<string, string>&) const override {
        ir.push_back("PUSH_STR " + label);
    }
};

struct LoadVarNode : AstNode {
    string name;
    explicit LoadVarNode(string n) : name(n) {}
    void generateIR(vector<string>& ir, unordered_map<string, int>&, unordered_map<string, string>&) const override {
        ir.push_back("LOAD " + name);
    }
};

struct StoreVarNode : AstNode {
    string name;
    unique_ptr<AstNode> expr;
    StoreVarNode(string n, unique_ptr<AstNode> e) : name(n), expr(move(e)) {}
    void generateIR(vector<string>& ir, unordered_map<string, int>& vars, unordered_map<string, string>& strings) const override {
        expr->generateIR(ir, vars, strings);
        ir.push_back("STORE " + name);
        if (!vars.count(name)) vars[name] = 1;
    }
};

struct BinaryOpNode : AstNode {
    TokenType op;
    unique_ptr<AstNode> left;
    unique_ptr<AstNode> right;
    BinaryOpNode(TokenType o, unique_ptr<AstNode> l, unique_ptr<AstNode> r)
            : op(o), left(move(l)), right(move(r)) {}

    void generateIR(vector<string>& ir, unordered_map<string, int>& vars, unordered_map<string, string>& strings) const override {
        left->generateIR(ir, vars, strings);
        right->generateIR(ir, vars, strings);
        switch (op) {
            case TokenType::Plus:     ir.push_back("ADD"); break;
            case TokenType::Minus:    ir.push_back("SUB"); break;
            case TokenType::Multiply: ir.push_back("MUL"); break;
            case TokenType::Divide:   ir.push_back("DIV"); break;
            case TokenType::Eq:   ir.push_back("EQ"); break;
            case TokenType::Neq:  ir.push_back("NEQ"); break;
            case TokenType::Lt:   ir.push_back("LT"); break;
            case TokenType::Gt:   ir.push_back("GT"); break;
            case TokenType::Lte:  ir.push_back("LTE"); break;
            case TokenType::Gte:  ir.push_back("GTE"); break;
            default: break;
        }
    }
};

struct PrintNode : AstNode {
    unique_ptr<AstNode> message;
    explicit PrintNode(unique_ptr<AstNode> msg) : message(move(msg)) {}
    void generateIR(vector<string>& ir, unordered_map<string, int>& vars, unordered_map<string, string>& strings) const override {
        message->generateIR(ir, vars, strings);
        if (message->isString()) {
            ir.push_back("PRINT_STR");
        } else {
            ir.push_back("PRINT_INT");
        }
    }
};

// ================== 语法分析器 (Parser) ==================
class Parser {
public:
    explicit Parser(Lexer lexer) : lexer(move(lexer)) {
        currentToken = this->lexer.nextToken();
    }
    vector<unique_ptr<AstNode>> parseProgram() {
        vector<unique_ptr<AstNode>> program;
        while (currentToken.type != TokenType::Eof) {
            program.push_back(parseStatement());
        }
        return program;
    }
    unordered_map<string, string> strings;
private:
    Token currentToken;
    Lexer lexer;
    void consume(TokenType type, const string& msg) {
        if (currentToken.type != type) {
            throw runtime_error(msg);
        }
        currentToken = lexer.nextToken();
    }
    unique_ptr<AstNode> parseStatement() {
        if (currentToken.type == TokenType::If) {
            return parseIf();
        } else if (currentToken.type == TokenType::While) {
            return parseWhile();
        } else if (currentToken.type == TokenType::Print) {
            return parsePrint();
        } else if (currentToken.type == TokenType::Identifier) {
            return parseAssignment();
        } else {
            throw runtime_error("Unexpected token in statement: " + currentToken.value);
        }
    }
    unique_ptr<AstNode> parsePrint() {
        consume(TokenType::Print, "Expected 'print'");
        consume(TokenType::LParen, "Expected '(' after print");
        auto expr = parseExpression();
        consume(TokenType::RParen, "Expected ')' after expression");
        consume(TokenType::Semicolon, "Expected ';' after print");
        return make_unique<PrintNode>(move(expr));
    }
    unique_ptr<AstNode> parseAssignment() {
        string varName = currentToken.value;
        consume(TokenType::Identifier, "Expected identifier");
        consume(TokenType::Assign, "Expected '=' after identifier");
        auto expr = parseExpression();
        consume(TokenType::Semicolon, "Expected ';' after expression");
        return make_unique<StoreVarNode>(varName, move(expr));
    }
    unique_ptr<AstNode> parseExpression() {
        return parseLogicalOr();
    }
    unique_ptr<AstNode> parseLogicalOr() {
        auto left = parseLogicalAnd();
        while (currentToken.type == TokenType::Eq || currentToken.type == TokenType::Neq) {
            TokenType op = currentToken.type;
            currentToken = lexer.nextToken();
            auto right = parseLogicalAnd();
            left = make_unique<BinaryOpNode>(op, move(left), move(right));
        }
        return left;
    }//
    unique_ptr<AstNode> parseLogicalAnd() {
        auto left = parseEquality();
        while (false) {} // 暂时不处理逻辑与
        return left;
    }//
    unique_ptr<AstNode> parseEquality() {
        auto left = parseRelational();
        while (currentToken.type == TokenType::Eq || currentToken.type == TokenType::Neq) {
            TokenType op = currentToken.type;
            currentToken = lexer.nextToken();
            auto right = parseRelational();
            left = make_unique<BinaryOpNode>(op, move(left), move(right));
        }
        return left;
    }//
    unique_ptr<AstNode> parseRelational() {
        auto left = parseAdditive();
        while (currentToken.type == TokenType::Lt || currentToken.type == TokenType::Gt ||
               currentToken.type == TokenType::Lte || currentToken.type == TokenType::Gte) {
            TokenType op = currentToken.type;
            currentToken = lexer.nextToken();
            auto right = parseAdditive();
            left = make_unique<BinaryOpNode>(op, move(left), move(right));
        }
        return left;
    }//
    unique_ptr<AstNode> parseAdditive() {
        auto left = parseMultiplicative();
        while (currentToken.type == TokenType::Plus || currentToken.type == TokenType::Minus) {
            TokenType op = currentToken.type;
            currentToken = lexer.nextToken();
            auto right = parseMultiplicative();
            left = make_unique<BinaryOpNode>(op, move(left), move(right));
        }
        return left;
    }
    unique_ptr<AstNode> parseMultiplicative() {
        auto left = parsePrimary();
        while (currentToken.type == TokenType::Multiply || currentToken.type == TokenType::Divide) {
            TokenType op = currentToken.type;
            currentToken = lexer.nextToken();
            auto right = parsePrimary();
            left = make_unique<BinaryOpNode>(op, move(left), move(right));
        }
        return left;
    }
    unique_ptr<AstNode> parseIf() {
        consume(TokenType::If, "Expected 'if'");
        consume(TokenType::LParen, "Expected '(' after 'if'");
        auto cond = parseExpression();
        consume(TokenType::RParen, "Expected ')' after condition");
        consume(TokenType::LBrace, "Expected '{'");
        vector<unique_ptr<AstNode>> thenBody;
        while (currentToken.type != TokenType::RBrace) {
            thenBody.push_back(parseStatement());
        }
        consume(TokenType::RBrace, "Expected '}' after then block");

        vector<unique_ptr<AstNode>> elseBody;
        if (currentToken.type == TokenType::Else) {
            consume(TokenType::Else, "Expected 'else'");
            consume(TokenType::LBrace, "Expected '{'");
            while (currentToken.type != TokenType::RBrace) {
                elseBody.push_back(parseStatement());
            }
            consume(TokenType::RBrace, "Expected '}' after else block");
        }

        return make_unique<IfNode>(move(cond), move(thenBody), move(elseBody));
    }
    unique_ptr<AstNode> parseWhile() {
        consume(TokenType::While, "Expected 'while'");
        consume(TokenType::LParen, "Expected '(' after 'while'");
        auto cond = parseExpression();
        consume(TokenType::RParen, "Expected ')' after condition");
        consume(TokenType::LBrace, "Expected '{'");
        vector<unique_ptr<AstNode>> body;
        while (currentToken.type != TokenType::RBrace) {
            body.push_back(parseStatement());
        }
        consume(TokenType::RBrace, "Expected '}' after loop body");
        return make_unique<WhileNode>(move(cond), move(body));
    }
    unique_ptr<AstNode> parsePrimary() {
        if (currentToken.type == TokenType::Number) {
            auto node = make_unique<NumberNode>(currentToken.value);
            currentToken = lexer.nextToken();
            return node;
        } else if (currentToken.type == TokenType::Identifier) {
            string name = currentToken.value;
            currentToken = lexer.nextToken();
            return make_unique<LoadVarNode>(name);
        } else if (currentToken.type == TokenType::String) {
            string val = currentToken.value;
            static int strCount = 0;
            string label = "msg" + to_string(strCount++);
            strings[label] = val;
            currentToken = lexer.nextToken();
            return make_unique<StringNode>(label);
        }
        else if (currentToken.type == TokenType::LParen) {
            currentToken = lexer.nextToken();
            auto expr = parseExpression();
            consume(TokenType::RParen, "Expected ')'");
            return expr;
        } else {
            throw runtime_error("Unexpected token: " + currentToken.value);
        }
    }
};

// ================== IR 优化器（常量折叠） ==================
void optimizeIR(vector<string>& ir) {
    for (size_t i = 0; i + 2 < ir.size(); ++i) {
        if (ir[i].substr(0, 5) == "PUSH " &&
            ir[i+1].substr(0, 5) == "PUSH " &&
            (ir[i+2] == "ADD" || ir[i+2] == "SUB" || ir[i+2] == "MUL" || ir[i+2] == "DIV")) {
            int a = stoi(ir[i].substr(5));
            int b = stoi(ir[i+1].substr(5));
            int result = 0;
            if (ir[i+2] == "ADD") result = a + b;
            else if (ir[i+2] == "SUB") result = a - b;
            else if (ir[i+2] == "MUL") result = a * b;
            else if (ir[i+2] == "DIV") result = a / b;
            ir[i] = "PUSH " + to_string(result);
            ir.erase(ir.begin()+i+1, ir.begin()+i+3);
            i -= 3;
        }
    }
}

// ================== 数据段生成 ==================
void generateDataSection(ofstream& out, const unordered_map<string, int>& vars,
                         const unordered_map<string, string>& strings) {
    out << "section .data\n";
    for (const auto& s : strings) {
        out << "    " << s.first << " db \"" << s.second << "\"\n";
        out << "    " << s.first << "_len equ $ - " << s.first << "\n";
    }
    for (const auto& v : vars) {
        out << "    " << v.first << " dd 0\n";
    }
    out << "    int_buffer times 16 db 0\n";
    out << "    int_buffer_len equ 16\n";
}

// ================== 代码生成器 ==================
void generateTextSectionHeader(ofstream& out) {
    out << "section .text\n";
    out << "    global _start\n";
    out << "_start:\n";
}

void generateCodeFromIR(const vector<string>& ir, ofstream& out,
                        const unordered_map<string, string>& strings) {
    for (const auto& line : ir) {
        if (line.substr(0, 5) == "PUSH ") {
            out << "    ;; " << line << "\n";
            out << "    push dword " << line.substr(5) << "\n";
        }
        else if (line.substr(0, 9) == "PUSH_STR ") {
            string label = line.substr(10);
            out << "    ;; " << line << "\n";
            out << "    push dword " << "m"<<label << "_len\n";
            out << "    push dword " << "m"<<label << "\n";
        }
        else if (line.substr(0, 5) == "LOAD ") {
            string var = line.substr(5);
            out << "    ;; " << line << "\n";
            out << "    mov eax, [" << var << "]\n";
            out << "    push eax\n";
        }
        else if (line.find("JZ ") == 0) {
            out << "    pop eax\n";
            out << "    test eax, eax\n";
            out << "    jz " << line.substr(3) << "\n";
        }
        else if (line == "EQ") {
            out << "    pop edx\n";
            out << "    pop eax\n";
            out << "    cmp eax, edx\n";
            out << "    sete al\n";  // 设置 al 为 1 或 0
            out << "    movzx eax, al\n";
            out << "    push eax\n";
        }//
        else if (line == "NEQ") {
            out << "    pop edx\n";
            out << "    pop eax\n";
            out << "    cmp eax, edx\n";
            out << "    setne al\n";
            out << "    movzx eax, al\n";
            out << "    push eax\n";
        }
        else if (line == "LT") {
            out << "    pop edx\n";
            out << "    pop eax\n";
            out << "    cmp eax, edx\n";
            out << "    setl al\n";
            out << "    movzx eax, al\n";
            out << "    push eax\n";
        }
        else if (line == "GT") {
            out << "    pop edx\n";
            out << "    pop eax\n";
            out << "    cmp eax, edx\n";
            out << "    setg al\n";
            out << "    movzx eax, al\n";
            out << "    push eax\n";
        }
        else if (line == "LTE") {
            out << "    pop edx\n";
            out << "    pop eax\n";
            out << "    cmp eax, edx\n";
            out << "    setle al\n";
            out << "    movzx eax, al\n";
            out << "    push eax\n";
        }
        else if (line == "GTE") {
            out << "    pop edx\n";
            out << "    pop eax\n";
            out << "    cmp eax, edx\n";
            out << "    setge al\n";
            out << "    movzx eax, al\n";
            out << "    push eax\n";
        }//
        else if (line.substr(0, 6) == "STORE ") {
            string var = line.substr(6);
            out << "    ;; " << line << "\n";
            out << "    pop eax\n";
            out << "    mov [" << var << "], eax\n";
        }
        else if (line == "ADD") {
            out << "    ;; ADD\n";
            out << "    pop ebx\n";
            out << "    pop eax\n";
            out << "    add eax, ebx\n";
            out << "    push eax\n";
        }
        else if (line == "PRINT_STR") {
            out << "    ;; PRINT_STR\n";
//            string label = line.substr(10);
//            out << "    push dword " << label << "\n";
//            out << "    push dword " << label << "_len\n";

//            out << "    push dword msg0\n";
//            out << "    push dword msg0_len\n";
            out << "    pop ecx\n";  // str ptr
            out << "    pop edx\n";  // len


            out << "    mov ebx, 1\n"; // stdout
            out << "    mov eax, 4\n"; // sys_write
            out << "    int 0x80\n";
        }
        else if (line == "PRINT_INT") {
            out << "    ;; PRINT_INT\n";
            out << "    pop eax\n";
            out << "    mov edi, int_buffer\n";
            out << "    call int_to_string\n";
            out << "    mov ecx, int_buffer\n";
            out << "    mov edx, eax\n";
            out << "    mov ebx, 1\n";
            out << "    mov eax, 4\n";
            out << "    int 0x80\n";
        }
        else if (line.substr(0, 4) == "JMP ") {
            string label = line.substr(4);
            out << "    ;; JMP " << label << "\n";
            out << "    jmp " << label << "\n";
        }
        else if (line.substr(0, 6) == "LABEL ") {
            string label = line.substr(6);
            out << label << ":\n";
        }
    }
}

// ================== 辅助函数 ==================
void generateIntToStringFunction(ofstream& out) {
    out << R"(
int_to_string:
    mov ebx, 10
    mov ecx, 0
    test eax, eax
    jns .positive
    neg eax
    mov byte [edi], '-'
    inc edi
    inc ecx
.positive:
    mov esi, edi
    add esi, 15
    mov byte [esi], 0
.dec_loop:
    dec esi
    xor edx, edx
    div ebx
    add dl, '0'
    mov [esi], dl
    inc ecx
    test eax, eax
    jnz .dec_loop
    mov eax, ecx
    push ecx
    mov ecx, eax
    cld
    rep movsb
    pop eax
    ret
)";
}

void generateExitCode(ofstream& out) {
    out << R"(
exit:
    mov eax, 1
    xor ebx, ebx
    int 0x80
)";
}

// ================== 主函数 ==================
int main(int argc, char* argv[]) {
    string inputFilename = "input.txt";
    if (argc > 1) inputFilename = argv[1];

    ifstream inFile(inputFilename);
    ofstream outFile("output.asm");

    if (!inFile.is_open()) {
        cerr << "Failed to open input file: " << inputFilename << endl;
        return 1;
    }

    stringstream buffer;
    buffer << inFile.rdbuf();
    string source = buffer.str();

    try {
        Lexer lexer(source);
        Parser parser(move(lexer));
        vector<unique_ptr<AstNode>> program = parser.parseProgram();

        vector<string> ir;
        unordered_map<string, int> variables;
        for (const auto& stmt : program) {
            stmt->generateIR(ir, variables, parser.strings);
        }
        ir.push_back("JMP exit"); // 替换 CALL exit

        cout << "Generated IR:\n";
        for (const auto& line : ir) cout << "  " << line << "\n";

        optimizeIR(ir);

        generateDataSection(outFile, variables, parser.strings);
        generateTextSectionHeader(outFile);
        generateCodeFromIR(ir, outFile, parser.strings);
        generateIntToStringFunction(outFile);
        generateExitCode(outFile);

        cout << "Compilation successful! Output written to output.asm\n";
    } catch (const exception& e) {
        cerr << "Error: " << e.what() << endl;
        return 1;
    }

    return 0;
}
