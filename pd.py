import sigrokdecode as srd
from common.srdhelper import bitpack

'''
https://sigrok.org/wiki/Protocol_decoder_API
Bu protokol mfrc531 chipini analiz edebilmek için yazılmıştır.
Protokol Decoder(srd.Decoder) classının içinde olmak zorundadır,classda sırasıyla init metadata start ve decode fonksiyonları çalışmaktadır
 init:değişkenlerin sıfırlanıp set edildiği bölümdür.
 metadata: sr dosyasının içinde bulunan metadata dosyasını okuyan fonksiyondur
 start: protokolün programın içinde görselleştirme yapabilmesi için gereken kayıtlar diyebiliriz.burada görselleştirme out_ann ile yapılırken
  out_python ile protokülün üstüne eklenecek başka bir protokole input verilmiş olur
 decode:en son çalışan fonksiyondur sürekli çalışması ve anlık streamda yakalamalar yapabilmek için içinde while true dögüsü bulunur burada 
  istenilen şartları yakalamsı için self.wait fonksiyonu kullanılır.
Listeler:
 CmdRC:Bilinen sık kullanılan RC fonksiyonlarının adlandırılması için kullanılır.
 ConstMaskRegAddr:Rc işleminin maskeleme olup olmadığını kontrol etmek için kullanılan register listesi
 ConstRegAddr:Genel register adres listesi buradaki ile maskeleme registerleri aslında bir bütündür bir tarafta olan diğer tarafta olamaz.
Mantık:
 Writeraw dan sonra Readraw gelirse bu ReadRc dir Writeraw gelirse WriteRc dir.
 ReadRc yapılan adrese WriteRc yapılıyorsa yapılan işlem türü Mask işlemi olabilir.
'''

CmdRC = {
    '0100' : 'Terminate Running Command',
    '067F' : 'Disable Alll Interrupts',
    '077F' : 'Reset Interrupt Request',
    '2203' : 'Disable: RxCRC, TxCRC. Enable: Parity',
    '222C' : 'Enable: RxCRC, TxCRC. Disable: Parity',
    '220F' : 'Enable: RxCRC, TxCRC, Parity',
    '222C' : 'Enable: RxCRC, TxCRC. Disable: Parity',   
}
ConstMaskRegAddr = {
    '09' : 'RegControl',
    '11' : 'RegTxControl',
    '1A' : 'RegDecoderControl',
    '1F' : 'RegClockQControl',  
}
ConstRegAddr = {

    '00' : 'RegPage',
    '01' : 'RegCommand',
    '02' : 'RegFIFOData',
    '03' : 'RegPrimaryStatus',
    '04' : 'RegFIFOLength',
    '05' : 'RegSecondaryStatus',
    '06' : 'RegInterruptEn',
    '07' : 'RegInterruptRq',

    '0A' : 'RegErrorFlag',
    '0B' : 'RegCollPos',
    '0C' : 'RegTimerValue',
    '0D' : 'RegCRCResultLSB',
    '0E' : 'RegCRCResultMSB',
    '0F' : 'RegBitFraming',

    '12' : 'RegCwConductance',
    '13' : 'RegModConductance',
    '14' : 'RegCoderControl',  
    '15' : 'RegModWidth',
    '16' : 'RegModWidthSOF',
    '17' : 'RegTypeBFraming',

    '19' : 'RegRxControl1',
    '1B' : 'RegBitPhase',
    '1C' : 'RegRxThreshold',
    '1D' : 'RegBPSKDemControl',
    '1E' : 'RegRxControl2',

    '21' : 'RegRxWait',
    '22' : 'RegChannelRedundancy',
    '23' : 'RegCRCPresetLSB',
    '24' : 'RegCRCPresetMSB',
    '25' : 'RegTimeSlotPeriod',
    '26' : 'RegMfOutSelect',

    '29' : 'RegFIFOLevel',
    '2A' : 'RegTimerClock',
    '2B' : 'RegTimerControl',
    '2C' : 'RegTimerReload',
    '2D' : 'RegIRqPinConfig',

    '3A' : 'RegTestAnaSelect',
    '3C' : 'RegTestConfiguration',
    '3D' : 'RegTestDigiSelect',
    '3E' : 'RegTestEE',
    '3F' : 'RegTestDigiAccess',

}



class MException(Exception): #Exception fırlatmak için kullanılan fonksiyon pass ile protokolün yüklenmesi atlanır.
    pass

def normalize_time(t): # zamanı µ (mikro) cinsine çevirir
   return t * 1000.0 * 1000.0

def normalize_reg(key): # registerlerin kontrol fonksiyonu
    if key in ConstMaskRegAddr.keys():
       return ConstMaskRegAddr[key]
    elif key in ConstRegAddr.keys():
        return ConstRegAddr[key]
    else:
        return key
   
class Decoder(srd.Decoder):
    api_version = 3
    id = 'mfrc531'
    name = 'MFRC531'
    longname = 'NXP MFRC531 Sync Bus'
    desc = 'NXP MFRC531 Sync Bus.'
    license = 'gplv2+'
    inputs = ['logic']
    outputs = ['mfrc531']
    optional_channels = ( #input listesi buradaki listenin indisi wait fonksiyonunda kullanılır.
         {'id': 'd0', 'name': 'D0', 'desc': 'Data line 0' },
         {'id': 'd1', 'name': 'D1', 'desc': 'Data line 1' },
         {'id': 'd2', 'name': 'D2', 'desc': 'Data line 2' },
         {'id': 'd3', 'name': 'D3', 'desc': 'Data line 3' },
         {'id': 'd4', 'name': 'D4', 'desc': 'Data line 4' },
         {'id': 'd5', 'name': 'D5', 'desc': 'Data line 5' },
         {'id': 'd6', 'name': 'D6', 'desc': 'Data line 6' },
         {'id': 'd7', 'name': 'D7', 'desc': 'Data line 7' },
         {'id': 'cs', 'name': 'CS', 'desc': 'Chip select line' },
         {'id': 'ale','name': 'ALE','desc': 'Adress latch enable line' },
         {'id': 'rd', 'name': 'RD', 'desc': 'Read clock line' },
         {'id': 'wr', 'name': 'WR', 'desc': 'Write clock line' },
    )
    options = (
        {'id': 'itemwidth',    'desc': 'Item Width in Samples',  'default': 750},#Yapılan görsellemenin kaç örnek genişliğinde olucağını belirler.
        {'id': 'showsamplenr', 'desc': 'Show Sample Number',     'default': 'no', 'values':('no', 'yes')},
        {'id': 'showrwdata',   'desc': 'Show Raw Data Info',     'default': 'yes', 'values':('no', 'yes')},
    )
    annotations = ( # görselleme isimleri ,listenin indisi önemli put fonksiyonunda kullanılır.
        ('rdRaw',  'ReadRaw'),
        ('wrRaw',  'WriteRaw'),
        ('info',   'Info'),
        ('warning','Warning'),
        ('error',  'Error'),
        ('rdRC',   'ReadRC'),
        ('wrRC',   'WriteRC'),
        ('rcCmd',  'RcCommandInfo'),
        ('rwDataInfo',  'RawDataInfo'),


    )
    annotation_rows = ( # annotationsları gruplayıp aynı satırda gösterebilmek içindir aynı satırda farklı annotationsları farklı renklerde gösterilir.
        ('logMsg',      'Logger Message', (2,3,4,)),
        ('rawData',     'Raw Data', (0,1,)),
        ('rawDataInfo', 'Raw Data Info', (8,)),
        ('rcData',      'RC Data', (5,6,)),
        ('rcCmdInfo',   'RC Cmd Info', (7,)),
    )

    def __init__(self):
        self.reset()

    def reset(self):
        self.samplerate = None
        self.mask = False
        self.address = 0
        self.rc = False
        self.samplenumrc=0
        self.rcChecknumb=0
        self.samplenumMask=0
        self.sampleNumALE=0
        self.input=''
        self.regAddr=''

    def metadata(self, key, value):
        if key == srd.SRD_CONF_SAMPLERATE:
            self.samplerate = value

    def start(self):
        self.out_python = self.register(srd.OUTPUT_PYTHON)
        self.out_ann = self.register(srd.OUTPUT_ANN)

    # görseli ekleyen fonksiyondur (başlangıç numarası,bitiş numarası,outann veya out python(bizde kullanılmıyor),annations indis ve yazılacak text).
    def putAnn(self,start,data):
        self.put(start,self.samplenum + self.options['itemwidth'] , self.out_ann, data) 

    #Readraw veya Writeraw da ale ve cs low olmak zorundadır alınan örneklerde kayma veya yanlış varmı diye kontrol edilir.
    def handle_control_bits(self, pins):
         log = ''

         if pins[9] !=0 :
            log += "ALE must be low. "

         if pins[8] !=0 :
            log += "CS must be low. "

         if len(log) > 0 :
            log += 'SampleNr:%d' % (self.samplenum) 
            self.putAnn(self.samplenum, [4, [log]])


    def handle_read_data(self, pins):
         self.rcChecknumb +=1 # mask işlemi için gereken rc işlemleri arasında başka işlem varmı kontrol sayacı
         self.handle_control_bits(pins)
         rdata= "{{:0{}X}}".format(2).format(bitpack(pins[0:8]))
         addr = "{{:0{}X}}".format(2).format(self.address & 0xFF);
         result = '%s=ReadRwRC(%s)' % ( rdata,addr) 
         self.putAnn(self.samplenum,[0, [result]])

         if self.options['showrwdata'] == 'yes':
            result = 'Reg: %s' % (normalize_reg(addr)) 
            self.putAnn(self.samplenum, [8, [result]])

         if self.options['showsamplenr'] == 'yes':
            result = 'SampleNr:%d' % (self.samplenum) 
            self.putAnn(self.samplenum, [2, [result]])

         if self.rc:# Rc mi kontrolü
            self.rc = False
            samples = self.samplenum - self.samplenumrc
            t = samples / self.samplerate
            time =normalize_time(t)

            if time > 6:#readrc 6 µs dan fazla olamaz
               if self.rcChecknumb >=2:
                  self.mask=False
                  self.rcChecknumb=0
               return

            addr = "{{:0{}X}}".format(2).format(self.address);
            result = '%s=ReadRC(%s)' % ( rdata,normalize_reg(addr))
            self.putAnn(self.samplenumrc, [5, [result]])
            if addr in ConstMaskRegAddr.keys() and not self.mask: # mask kontrolü
                self.regAddr=addr
                self.input=rdata
                self.samplenumMask=self.samplenumrc
                self.mask=True
            else:
                self.mask=False 
            self.rcChecknumb=0
         else:   
            if self.rcChecknumb >=2:
               self.mask=False
               self.rcChecknumb=0

    
    def handle_write_data(self, pins):
        self.handle_control_bits(pins)
        self.rcChecknumb +=1 
        wdata= "{{:0{}X}}".format(2).format(bitpack(pins[0:8]))
        addr = "{{:0{}X}}".format(2).format(self.address & 0xFF);
        result = 'WriteRwRC(%s,%s)' % ( addr,wdata) 
        self.putAnn(self.samplenum, [1, [result]])

        if self.options['showrwdata'] == 'yes':
            result = 'Reg: %s' % (normalize_reg(addr)) 
            self.putAnn(self.samplenum, [8, [result]])
         
        if self.options['showsamplenr'] == 'yes':
            result = 'SampleNr:%d' % (self.samplenum) 
            self.putAnn(self.samplenum, [2, [result]])
         
        if self.rc:#Rc doğrulandı mı ?
            samples = self.samplenum - self.samplenumrc
            t = samples / self.samplerate
            time =normalize_time(t)

            if time > 7:#writerc 7 µs dan fazla olamaz
               self.samplenumrc=self.samplenum
               if self.rcChecknumb >=2:
                  self.mask=False
                  self.rcChecknumb=0
               return 

            self.rc = False
            addr = "{{:0{}X}}".format(2).format(self.address);
            result = 'WriteRC(%s,%s)' % ( normalize_reg(addr),wdata)
            self.putAnn(self.samplenumrc, [6, [result]])
            # ====================================================
            key = addr+wdata
            if key in CmdRC.keys() :#Bilnen fonksiyon isimlerinin kontrolü
               result = CmdRC[key]
               self.putAnn(self.samplenumrc, [7, [result]])

               # output = input işlem mask
               # mask = input ^ output 
               # if input & mask == 00 :=> setbitmask durumu
               # if input | mask == FF :=> clearbitmask durumu 

            if self.mask and addr == self.regAddr and self.rcChecknumb <=2: # Mask kontrolü
               mask = int(self.input,base=16) ^ int(wdata,base=16)
               if (int(self.input,base=16) & mask) == 0: # işlem set bit mask
                 result = 'SetBitMask(%s,%s)' % ( normalize_reg(addr),"{{:0{}X}}".format(2).format(mask))
                 self.putAnn(self.samplenumMask, [7, [result]])

               elif (int(self.input,base=16) | mask) == 255:# işlem clear bit mask
                  result = 'ClearBitMask(%s,%s)' % ( normalize_reg(addr),"{{:0{}X}}".format(2).format(mask))
                  self.putAnn(self.samplenumMask, [7, [result]])

               else:#hatalı işlem
                result = 'Hatalı bit mask(%s,%s)' % ( normalize_reg(addr),"{{:0{}X}}".format(2).format(mask))
                self.putAnn(self.samplenumMask, [4, [result]])
               
               self.mask=False
               self.rcChecknumb=0

        elif addr=='00' and (int(wdata,base=16) > int('7F',base=16)):# yazılan adres 00 ve yazılan değer 7f den büyükse Rc işlemidir
         self.rc = True
         self.samplenumrc=self.samplenum

        if self.rcChecknumb >=2:
           self.mask=False
           self.rcChecknumb=0
            


    def decode(self):
        if not self.has_channel(8):
           raise MException('CS must be selected')

        if not self.samplerate:
            raise MException('Cannot decode without samplerate.')

        while True:
            # 8:CS  9:ALE  10:RD  11:WR
            pins = self.wait([{9: 'r'},{9: 'f'},{10: 'r'},{11: 'r'}]) # pinlerin stateleri raise fall low high stable either veya pin yerine pin skip kullanılabilir
            #pins sağlanan şart anındaki tüm input pinlerinin değerlerini verir.
            if self.matched[0] :# ale raise durumu
               self.sampleNumALE= self.samplenum

            if self.matched[1] and (self.samplenum-self.sampleNumALE) > 2 :# ale fall durumu ve raise ile fall arası en az 2 örnek olmak zorund(sağlıklı bir veri için gerekli)
               self.address= bitpack(pins[0:8]) #ale her fall olduğunda işlem yapılacak registerin adresi alınır

            if self.matched[2] and pins[8]==0 : 
               self.handle_read_data(pins)

            if self.matched[3] and pins[8]==0 :
               self.handle_write_data(pins)

