import {defineConfig} from 'vite';


export default defineConfig(({mode}) => {
    return {
        base: mode == 'production' ? '/mrmp_introduction' : '/',
        server: {
            host: true,
            port: 3000,
            fs: {
                // 알고리즘 페이지가 저장소의 실제 소스(python/, cpp/)를 ?raw 로 import 한다.
                // '..' 은 vite root(document/) 기준 저장소 루트다.
                allow: ['..']
            }
        }
    }
});
