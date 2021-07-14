from socket import socket, AF_INET, SOCK_STREAM, timeout
import socketserver
from socket import gethostname, gethostbyname


corpusText = 'the of and to in that it is for you was with on as have but be they are at by he not this we or do from a an so were know like all there his about if has what just yeah my had would more when can who out which said their no she up been think well than some will because other did new me time her them also people get right these now then could only really into how see your most good after even much our here where over him may go any first years way those too very many going should such got year work make say back last little still between something mean both each lot before being same through thing want use while times down things take cells made off day kind data us never since might world another different using around why house need though long during number part home state life big come york found american under information president however own percent every probably high study always story better thought few look less against government show end find actually great family put point old sure news money clinton states week least school genes real figure case yes pretty today place system without next best city children course analysis am pm expression cell group set again public small gene let ever went whether must maybe results company once guess although days anything either second several women book national control doing enough fact having love human until far man patients service important among united care bad name help away done read feel getting program yet white give business report stuff country example whole problem seen early within makes ago came often left federal general protein almost water rather night washington keep large change war true call major hard site tell known studies health someone law movie possible question already making table post area able play legal took similar else office kids bush sequence saying everything market shown center along later bit sort angel test become buffy idea pay person order run bill power type past security level age paper job levels car words try john support according believe nothing street together nice start others live side word effect death recent likely top men remember talk means reported black hours significant sense services quite political cost guy months especially single present half available history companies young room changes model season head sometimes research local america cases air department talking rate reason town instead process issue evidence free role members across plan watch interesting open seem further additional saw proteins activity increase current problems described reports thus form close low usually follow everyone hand development provide music response south risk deal groups university former couple higher minutes anyone court comes goes buy page financial perhaps english act late whose specific texas value full lead team recently north heard taken mother taking front wrong above thinking interest stock child class result sequences party food include parents matter programs whatever gets involved standard spike himself game morning gonna growth month tax drug officials lost weeks common addition woman west associated clear future social function hope turn coming compared questions outside certain fun management friends art century average region itself effects summer blood individual final tv college third language disease difference friend understand anyway issues short stop cancer culture cut observed move computer congress list review administration special exactly behind except expected july interview required ones potential methods face community military god living particular international body identified film values normal policy finally economic campaign due themselves quality performance therefore books personal rule period population wall heart building terms lower related main private father slate added director points became action subject answer position medical amount hear happy red wife church longer article students piece technology size media leave press ask view performed near central agency inside east industry gone americans stories senate costs section approach space worth cause hit board binding began complex previously british experience sites sex french strong series rest soon nearly poor husband ways note web budget light expressed david cannot force lines journal despite design nation agree internet independent fine various monday cover committee toward education capital executive bring easy wrote myself middle stay star park reading difficult earlier tuesday decision one alone hour miles differences range television everybody original conditions chief provided island dead visit museum attention please fall presence greater certainly species products built particularly sunday guys address plus price term running rates complete trade police sounds dollars road record product simply file field method association consider factors structure hot california brought son boy funny generally samples organization numbers benefits character moment foreign happen meet drive watching girl economy somebody writing evil rights papers spent mail determine friday events share meeting phone mice popular supposed george upon sound ability access families justice chance discussion written staff previous significantly corporate production wednesday angeles spend domain republican patient entire received primary daily episode completely rules relationship positive training suggest china version key situation thursday sample drugs containing green march project factor modern unless obtained published multiple schools lack michael cool return tissue vote favorite mostly collection brain below credit indeed fire society places huge beyond energy died sorry defense hair source enjoy base highly decided click ok loss changed hold countries older thanks check developed background june experiments trial clearly basis allowed offer effective eat natural apparently directly stage wait paid race remains nuclear account kept necessary distribution official continue choice reaction door write wonder volume none cold step kill land serious weight intelligence magazine picture bought starting allow acid western ground baby software concentration king mine bank beginning rock network appears clinical genome send simple practice proposed speech beach efforts learn member nature attack overall pressure release blue appropriate unit limited walk clients dear chinese eyes effort insurance direct comment box contrast income reason commission appear relative hospital initial player equal target employee knowledge create pick critics win hello fish consistent measure senior'

def getPrompt(arr):
    while len(arr) < 9:
        arr.append('___')
    ret = '\n'
    for i in range(len(arr)):
        ret += ('{:^18}'.format(arr[i]))
        if i % 3 == 2:
            ret += '\n'
    return ret


class CursorServer:
    def __init__(self, server_addr, port, timeout=1):
        self.my_socket = socket(AF_INET, SOCK_STREAM)
        self.my_socket.settimeout(timeout)
        self.my_socket.setsockopt(SOL_SOCKET, SO_REUSEADDR, 1)
        # self.connect()
        self.my_socket.bind((server_addr, port))
        self.my_socket.listen()

    def __enter__(self):
        return self

    def __exit__(self, type, value, traceback):
        self.close()

    def connect(self, address, port):
        self.my_socket.connect((address, port))
        print(f"client connecting to server: {address}")

    def close(self):
        self.my_socket.close()
        print("remote client socket closed")

    def send(self, touch_state, x, y):
        ## touch state: 1按下，2按下时移动，3松开，4松开时移动
        paras = [touch_state, x, y]
        self.my_socket.send((" ".join([str(item) for item in paras])+"\n").encode())
        print(f"send: {paras}")


class MyHandler(socketserver.BaseRequestHandler):
    def handle(self):
        # global connect_request
        print('connected!')

        # self.data = self.request.recv(1024).strip()
        # print("{} wrote:".format(self.client_address[0]))
        # print(self.data)

        corpus = corpusText.split(' ')
        assert(len(corpus) == 1000)
        keychars = ['abc', 'def', 'ghi', 'jkl', 'mno', 'pqrs', 'tuv', 'wxyz']
        code = ''
        candidates = []
        inputText = ''

        while True:
            paras = self.request.recv(1024).decode().strip().split()
            if not paras:
                continue
            print(paras)
            print("USER INPUT:" + inputText + "_")
            select = len(candidates) > 0 and len(candidates) <= 9
            if select:
                prompt = getPrompt(candidates)
            else:
                prompt = getPrompt(keychars)
            if len(code) > 0 and len(candidates) == 0:
                print("CLEAR")
                code = ''
            print(prompt)
            key = paras[0]
            # key = input(prompt)
            if key == 'q':
                break
            elif key in '0123456789':
                if select:
                    inputText += (candidates[int(key)] + ' ')
                    candidates = []
                    code = ''
                else:
                    code += key
                    candidates = []
                    for word in corpus:
                        if len(word) >= len(code):
                            ok = True
                            for i in range(len(code)):
                                if word[i] not in keychars[int(code[i])]:
                                    ok = False
                                    break
                            if ok:
                                candidates.append(word)
                    print(candidates)


HOST = "localhost"
PORT = 23456


if __name__ == '__main__':
    # corpus = corpusText.split(' ')
    # assert(len(corpus) == 1000)
    # keychars = ['abc', 'def', 'ghi', 'jkl', 'mno', 'pqrs', 'tuv', 'wxyz']
    # code = ''
    # candidates = []
    # inputText = ''

    with socketserver.TCPServer((HOST, PORT), MyHandler) as server:
        # server.my_client = my_client
        print("Start to serve forever...")
        server.serve_forever()

    # while True:
    #     print("USER INPUT:" + inputText + "_")
    #     select = len(candidates) > 0 and len(candidates) <= 9
    #     if select:
    #         prompt = getPrompt(candidates)
    #     else:
    #         prompt = getPrompt(keychars)
    #     key = input(prompt)
    #     if key == 'q':
    #         break
    #     elif key in '0123456789':
    #         if select:
    #             inputText += (candidates[int(key)] + ' ')
    #             candidates = []
    #             code = ''
    #         else:
    #             code += key
    #             candidates = []
    #             for word in corpus:
    #                 if len(word) >= len(code):
    #                     ok = True
    #                     for i in range(len(code)):
    #                         if word[i] not in keychars[int(code[i])]:
    #                             ok = False
    #                             break
    #                     if ok:
    #                         candidates.append(word)
    #             print(candidates)