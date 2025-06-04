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
    Identifier,
    Number,
    String,
    Print,
    If,
    Else,
    While,
    Assign,
    Plus,
    Minus,
    Multiply,
    Divide,
    Eq,
    Neq,
    Lt,
    Gt,
    Lte,
    Gte,
    LParen,
    RParen,
    LBrace,
    RBrace,
    Semicolon,
    Eof
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
            if (ident == "else") return {TokenType::Else, ident};
            if (ident == "while") return {TokenType::While, ident};

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
                    // Handle escape sequences
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
            if (pos < input.size() && input[pos] == '"') {
                pos++; // skip closing "
            }
            return {TokenType::String, str};
        }

        switch (c) {
            case '=':
                pos++;
                if (pos < input.size() && input[pos] == '=') {
                    pos++;
                    return {TokenType::Eq, "=="};
                }
                return {TokenType::Assign, "="};
            case '!':
                pos++;
                if (pos < input.size() && input[pos] == '=') {
                    pos++;
                    return {TokenType::Neq, "!="};
                }
                break;
            case '<':
                pos++;
                if (pos < input.size() && input[pos] == '=') {
                    pos++;
                    return {TokenType::Lte, "<="};
                }
                return {TokenType::Lt, "<"};
            case '>':
                pos++;
                if (pos < input.size() && input[pos] == '=') {
                    pos++;
                    return {TokenType::Gte, ">="};
                }
                return {TokenType::Gt, ">"};
            case '+': pos++; return {TokenType::Plus, "+"};
            case '-': pos++; return {TokenType::Minus, "-"};
            case '*': pos++; return {TokenType::Multiply, "*"};
            case '/': pos++; return {TokenType::Divide, "/"};
            case '(': pos++; return {TokenType::LParen, "("};
            case ')': pos++; return {TokenType::RParen, ")"};
            case '{': pos++; return {TokenType::LBrace, "{"};
            case '}': pos++; return {TokenType::RBrace, "}"};
            case ';': pos++; return {TokenType::Semicolon, ";"};
            default:
                throw runtime_error("Unknown character: " + string(1, c));
        }
        return {TokenType::Eof, ""};
    }

private:
    string input;
    size_t pos;

    void skipWhitespace() {
        while (pos < input.size() && isspace(input[pos])) {
            pos++;
        }
    }
};

// ================== AST节点 ==================
struct AstNode {
    virtual ~AstNode() = default;
    virtual void generateIR(vector<string>& ir, unordered_map<string, int>& vars, unordered_map<string, string>& strings) const = 0;
    virtual bool isString() const { return false; }
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
        if (!vars.count(name)) {
            vars[name] = 1;
        }
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
            case TokenType::Eq:       ir.push_back("EQ"); break;
            case TokenType::Neq:      ir.push_back("NEQ"); break;
            case TokenType::Lt:       ir.push_back("LT"); break;
            case TokenType::Gt:       ir.push_back("GT"); break;
            case TokenType::Lte:      ir.push_back("LTE"); break;
            case TokenType::Gte:      ir.push_back("GTE"); break;
            default: break;
        }
    }
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
        ir.push_back("JZ ELSE_" + to_string(id));

        for (const auto& stmt : thenBody) {
            stmt->generateIR(ir, vars, strings);
        }

        ir.push_back("JMP END_IF_" + to_string(id));
        ir.push_back("LABEL ELSE_" + to_string(id));

        for (const auto& stmt : elseBody) {
            stmt->generateIR(ir, vars, strings);
        }

        ir.push_back("LABEL END_IF_" + to_string(id));
    }
};

struct WhileNode : AstNode {
    unique_ptr<AstNode> condition;
    vector<unique_ptr<AstNode>> body;

    WhileNode(unique_ptr<AstNode> cond, vector<unique_ptr<AstNode>> bd)
            : condition(move(cond)), body(move(bd)) {}

    void generateIR(vector<string>& ir, unordered_map<string, int>& vars, unordered_map<string, string>& strings) const override {
        static int labelId = 0;
        int id = labelId++;

        ir.push_back("LABEL LOOP_START_" + to_string(id));
        condition->generateIR(ir, vars, strings);
        ir.push_back("JZ LOOP_END_" + to_string(id));

        for (const auto& stmt : body) {
            stmt->generateIR(ir, vars, strings);
        }

        ir.push_back("JMP LOOP_START_" + to_string(id));
        ir.push_back("LABEL LOOP_END_" + to_string(id));
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
            throw runtime_error(msg + ". Expected: " +
                                to_string(static_cast<int>(type)) +
                                ", got: " + to_string(static_cast<int>(currentToken.type)));
        }
        currentToken = lexer.nextToken();
    }

    unique_ptr<AstNode> parseStatement() {
        if (currentToken.type == TokenType::Print) {
            return parsePrint();
        } else if (currentToken.type == TokenType::If) {
            return parseIf();
        } else if (currentToken.type == TokenType::While) {
            return parseWhile();
        } else if (currentToken.type == TokenType::Identifier) {
            return parseAssignment();
        } else if (currentToken.type == TokenType::LBrace) {
            return parseBlock();
        } else {
            throw runtime_error("Unexpected token: " + currentToken.value);
        }
    }

    unique_ptr<AstNode> parsePrint() {
        consume(TokenType::Print, "Expected 'print'");
        consume(TokenType::LParen, "Expected '(' after print");
        auto expr = parseExpression();
        consume(TokenType::RParen, "Expected ')' after expression");
        consume(TokenType::Semicolon, "Expected ';' after print");
        return make_unique<PrintNode>(move(expr)); // 使用 move
    }

    unique_ptr<AstNode> parseIf() {
        consume(TokenType::If, "Expected 'if'");
        consume(TokenType::LParen, "Expected '(' after if");
        auto cond = parseExpression();
        consume(TokenType::RParen, "Expected ')' after condition");

        auto thenBlock = parseBlock();

        vector<unique_ptr<AstNode>> elseBlock;
        if (currentToken.type == TokenType::Else) {
            consume(TokenType::Else, "Expected 'else'");
            elseBlock = move(parseBlock()->body); // 使用 move
        }

        return make_unique<IfNode>(move(cond), move(thenBlock->body), move(elseBlock)); // 使用 move
    }

    unique_ptr<WhileNode> parseWhile() {
        consume(TokenType::While, "Expected 'while'");
        consume(TokenType::LParen, "Expected '(' after while");
        auto cond = parseExpression();
        consume(TokenType::RParen, "Expected ')' after condition");
        return make_unique<WhileNode>(move(cond), move(parseBlock()->body)); // 使用 move
    }

    unique_ptr<AstNode> parseAssignment() {
        string varName = currentToken.value;
        consume(TokenType::Identifier, "Expected identifier");
        consume(TokenType::Assign, "Expected '=' after identifier");
        auto expr = parseExpression();
        consume(TokenType::Semicolon, "Expected ';' after expression");
        return make_unique<StoreVarNode>(varName, move(expr)); // 使用 move
    }

    unique_ptr<WhileNode> parseBlock() {
        consume(TokenType::LBrace, "Expected '{'");
        vector<unique_ptr<AstNode>> body;
        while (currentToken.type != TokenType::RBrace && currentToken.type != TokenType::Eof) {
            body.push_back(parseStatement()); // 这里没问题，因为 parseStatement() 返回的是右值
        }
        consume(TokenType::RBrace, "Expected '}'");
        return make_unique<WhileNode>(nullptr, move(body)); // 使用 move
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
            left = make_unique<BinaryOpNode>(op, move(left), move(right)); // 使用 move
        }
        return left;
    }

    unique_ptr<AstNode> parseLogicalAnd() {
        auto left = parseRelational();
        while (currentToken.type == TokenType::Lt || currentToken.type == TokenType::Gt ||
               currentToken.type == TokenType::Lte || currentToken.type == TokenType::Gte) {
            TokenType op = currentToken.type;
            currentToken = lexer.nextToken();
            auto right = parseRelational();
            left = make_unique<BinaryOpNode>(op, move(left), move(right));
        }
        return left;
    }

    unique_ptr<AstNode> parseRelational() {
        auto left = parseAdditive();
        while (currentToken.type == TokenType::Plus || currentToken.type == TokenType::Minus) {
            TokenType op = currentToken.type;
            currentToken = lexer.nextToken();
            auto right = parseAdditive();
            left = make_unique<BinaryOpNode>(op, move(left), move(right));
        }
        return left;
    }

    unique_ptr<AstNode> parseAdditive() {
        auto left = parseMultiplicative();
        while (currentToken.type == TokenType::Multiply || currentToken.type == TokenType::Divide) {
            TokenType op = currentToken.type;
            currentToken = lexer.nextToken();
            auto right = parseMultiplicative();
            left = make_unique<BinaryOpNode>(op, move(left), move(right));
        }
        return left;
    }

    unique_ptr<AstNode> parseMultiplicative() {
        auto left = parsePrimary();
        while (currentToken.type == TokenType::LParen || currentToken.type == TokenType::Number ||
               currentToken.type == TokenType::Identifier || currentToken.type == TokenType::String) {
            auto right = parsePrimary();
            left = make_unique<BinaryOpNode>(TokenType::Multiply, move(left), move(right));
        }
        return left;
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
        } else if (currentToken.type == TokenType::LParen) {
            currentToken = lexer.nextToken();
            auto expr = parseExpression();
            if (currentToken.type != TokenType::RParen) {
                throw runtime_error("Expected ')'");
            }
            currentToken = lexer.nextToken();
            return expr;
        } else {
            throw runtime_error("Unexpected token: " + currentToken.value);
        }
    }
};

// ================== 代码生成器 ==================
void generateDataSection(ofstream& out, const unordered_map<string, int>& vars,
                         const unordered_map<string, string>& strings) {
    out << "section .data\n";

    // 字符串常量
    for (const auto& s : strings) {
        out << "    " << s.first << " db \"" << s.second << "\", 0\n";
        out << "    " << s.first << "_len equ $ - " << s.first << "\n";
    }

    // 变量
    for (const auto& v : vars) {
        out << "    " << v.first << " dd 0\n";
    }

    // 整数打印缓冲区
    out << "    int_buffer times 16 db 0\n";
    out << "    int_buffer_len equ 16\n\n";
}

void generateTextSectionHeader(ofstream& out) {
    out << "section .text\n";
    out << "    global _start\n";
    out << "    extern strlen\n\n";
    out << "_start:\n";
}

void generateCodeFromIR(const vector<string>& ir, ofstream& out,
                        const unordered_map<string, string>& strings) {
    stack<string> regStack;
    int labelCounter = 0;

    for (const auto& line : ir) {
        if (line.substr(0, 5) == "PUSH ") {
            string val = line.substr(5);
            out << "    ;; " << line << "\n";
            out << "    push dword " << val << "\n";
        }
        else if (line.substr(0, 10) == "PUSH_STR ") {
            string label = line.substr(10);
            out << "    ;; " << line << "\n";
            out << "    push dword " << label << "\n";
            out << "    push dword " << label << "_len\n";
        }
        else if (line.substr(0, 5) == "LOAD ") {
            string var = line.substr(5);
            out << "    ;; " << line << "\n";
            out << "    mov eax, [" << var << "]\n";
            out << "    push eax\n";
        }
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
        else if (line == "SUB") {
            out << "    ;; SUB\n";
            out << "    pop ebx\n";
            out << "    pop eax\n";
            out << "    sub eax, ebx\n";
            out << "    push eax\n";
        }
        else if (line == "MUL") {
            out << "    ;; MUL\n";
            out << "    pop ebx\n";
            out << "    pop eax\n";
            out << "    imul eax, ebx\n";
            out << "    push eax\n";
        }
        else if (line == "DIV") {
            out << "    ;; DIV\n";
            out << "    pop ebx\n";
            out << "    pop eax\n";
            out << "    cdq\n";
            out << "    idiv ebx\n";
            out << "    push eax\n";
        }
        else if (line == "EQ") {
            out << "    ;; EQ\n";
            out << "    pop ebx\n";
            out << "    pop eax\n";
            out << "    cmp eax, ebx\n";
            out << "    sete al\n";
            out << "    movzx eax, al\n";
            out << "    push eax\n";
        }
        else if (line == "NEQ") {
            out << "    ;; NEQ\n";
            out << "    pop ebx\n";
            out << "    pop eax\n";
            out << "    cmp eax, ebx\n";
            out << "    setne al\n";
            out << "    movzx eax, al\n";
            out << "    push eax\n";
        }
        else if (line == "LT") {
            out << "    ;; LT\n";
            out << "    pop ebx\n";
            out << "    pop eax\n";
            out << "    cmp eax, ebx\n";
            out << "    setl al\n";
            out << "    movzx eax, al\n";
            out << "    push eax\n";
        }
        else if (line == "GT") {
            out << "    ;; GT\n";
            out << "    pop ebx\n";
            out << "    pop eax\n";
            out << "    cmp eax, ebx\n";
            out << "    setg al\n";
            out << "    movzx eax, al\n";
            out << "    push eax\n";
        }
        else if (line == "PRINT_STR") {
            out << "    ;; PRINT_STR\n";
            out << "    pop edx    ; len\n";
            out << "    pop ecx    ; str ptr\n";
            out << "    mov ebx, 1 ; stdout\n";
            out << "    mov eax, 4 ; sys_write\n";
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
        else if (line.substr(0, 3) == "JZ ") {
            string label = line.substr(3);
            out << "    ;; " << line << "\n";
            out << "    pop eax\n";
            out << "    test eax, eax\n";
            out << "    jz " << label << "\n";
        }
        else if (line.substr(0, 4) == "JMP ") {
            string label = line.substr(4);
            out << "    ;; " << line << "\n";
            out << "    jmp " << label << "\n";
        }
        else if (line.substr(0, 6) == "LABEL ") {
            string label = line.substr(6);
            out << label << ":\n";
        }
    }
}

void generateIntToStringFunction(ofstream& out) {
    out << R"(
; Convert integer in EAX to string, store in EDI
; Returns length in EAX
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
    add esi, 15  ; end of buffer
    mov byte [esi], 0 ; null terminator
.dec_loop:
    dec esi
    xor edx, edx
    div ebx
    add dl, '0'
    mov [esi], dl
    inc ecx
    test eax, eax
    jnz .dec_loop

    ; Copy result to buffer
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
    if (argc > 1) {
        inputFilename = argv[1];
    }

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

        // Generate IR
        vector<string> ir;
        unordered_map<string, int> variables;



        for (const auto& stmt : program) {
            stmt->generateIR(ir, variables, parser.strings);
        }
        // 添加退出调用
        ir.push_back("CALL exit");
        cout << "Parsed " << program.size() << " statements." << endl;
        // Optimize IR
        cout << "Generated IR:" << endl;
        for (const auto& line : ir) {
            cout << "  " << line << endl;
        }
        optimizeIR(ir);

        // Generate assembly
        generateDataSection(outFile, variables, parser.strings);
        generateTextSectionHeader(outFile);
        generateCodeFromIR(ir, outFile, parser.strings);
        generateIntToStringFunction(outFile);
        generateExitCode(outFile);

        cout << "Compilation successful! Output written to output.asm" << endl;
    } catch (const exception& e) {
        cerr << "Error: " << e.what() << endl;
        return 1;
    }

    return 0;
}
