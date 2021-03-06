# -*- coding: utf-8 -*-
import requests,os,sys,json,random
from datetime import datetime

now = datetime.now()
dt = now.strftime("%d/%m/%Y %H:%M:%S")
kek = now.strftime("%d_%h_%Y_%H_%M")
print(kek)
token = os.getenv('VK_TOKEN')
peer = 2000000025
attachs=''

def vk(method,**kwargs):
	param = {'v':'5.91','access_token':token}
	param.update(kwargs)
	ret = requests.post('https://api.vk.com/method/'+method, data=param)
	return ret.json()

srv=vk('docs.getMessagesUploadServer', peer_id=peer)

for i in sys.argv[2:]:
	title=i.split('.')
	title=title[0]+'_'+kek+'.'+title[1]
	os.rename(i, i+i[-1])
	with open(i+i[-1], 'r') as f:
		br = json.loads(requests.post(srv['response']['upload_url'],files={'file': f}).text)
		doc=vk('docs.save',file=br['file'],title=title)['response']['doc']
		attachs+='doc'+str(doc['owner_id'])+'_'+str(doc['id'])+','
	os.rename(i+i[-1], i)


vk('messages.send', peer_id=peer, message='Commit message: '+sys.argv[1]+'\nBuild finished at '+dt, attachment=attachs,random_id=random.randint(0,2**10))
